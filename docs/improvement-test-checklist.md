# mycobot MCP/REST 改善 — 実機テストチェックリスト

[docs/test-report.md](test-report.md) の指摘＋公式ドキュメント照合で入れた改善を、実機で確認するための項目。
各項目に「変更点 / 手順 / 合否基準」を記載。レポート §1 の表で ❌/△ だった項目が ✅ になることがゴール。

- 対象: REST API `http://192.168.0.136:8080`（必要に応じて IP を読み替え）
- グリッパーは parallel（`gripper_type=3`）想定
- 速度は安全のため低速（`speed=20〜30`）推奨

## 0. 準備

- [ ] サーバー起動: `./mycobot_server_ctl.sh start`（`/health` 通過まで待つ）
- [ ] `curl -s http://192.168.0.136:8080/health` が `{"status":"healthy","robot_connected":true,...}`
- [ ] 起動ログ（stderr）に異常なし。`fresh_mode` 設定でエラーが出ていない
- [ ] 周囲に干渉物がない・非常停止に手が届く状態

---

## A. 明確なバグ修正

### A-1. `jog_joint` のハング解消（最優先 / レポート 2.1）
変更: 連続ジョグ `jog_angle` を廃止し、現在角＋increment を `send_angle` で送って即リターン。target は関節リミットにクランプ。

- [ ] `curl -s -X POST http://192.168.0.136:8080/joints/1/jog -H 'Content-Type: application/json' -d '{"direction":1,"speed":20,"increment":5}'`
  - 合否: **即座に**（数秒以内）200 が返る／ハングしない
  - 合否: メッセージに実 target（例 `to 24.77°`）が入る
  - 合否: 直後 `GET /joints/1/angle` で J1 が約 +5° 増えている
- [ ] `direction":-1` で逆方向に約5°戻る
- [ ] increment 範囲外（`"increment":0` / `100`）→ 422 で弾かれる
- [ ] リミット境界付近で increment 超過 → target がリミットにクランプされる（メッセージの to 値で確認）

### A-2. `release`（グリッパー）の flag バグ（レポート 2.2）
変更: 公開値 `10`→ ファーム flag `254` に写像（open=0/close=1 は不変）。

- [ ] `curl -s -X POST http://192.168.0.136:8080/gripper/release -H 'Content-Type: application/json' -d '{"gripper_type":3}'`
  - 合否: **200**（以前の 503 "flag is 0 or 1 or 254 … received 10" が出ない）／グリッパーがトルクオフ
- [ ] `open` / `close` が従来どおり正常（リグレッション確認）

---

## B. 完了判定 `wait_for_completion`（レポート 3）
変更: 許容誤差(tol)収束 / ストール検知 / タイムアウト の3終了条件＋失敗時 auto-stop。戻り値に `reason` / `max_error`。目標は last-write-wins でサーバー保持。

- [ ] 収束（成功）: `PUT /joints/angles {"angles":[20,0,0,0,0,0],"speed":30}` の直後に
  `curl -s -X POST http://192.168.0.136:8080/robot/wait -H 'Content-Type: application/json' -d '{"tolerance":1.0,"timeout":15}'`
  - 合否: `completed:true` / `reason:"converged"` / `max_error <= 1.0` を返す
- [ ] go_home でも収束: `POST /robot/home {"speed":30}` → `POST /robot/wait` が `converged`
- [ ] タイムアウト時に**実機が止まる**: 大移動を低速で開始し、短い timeout（例 `{"timeout":1.0}`）で wait
  - 合否: `reason:"timeout"` 返却後、ロボットが停止している（以前は動き続けた）
- [ ] ストール検知（任意 / 安全に再現できる場合のみ）: 物理的に届かない状況で `reason:"stalled"` 返却＋停止
- [ ] target 明示指定でも動く: `POST /robot/wait {"target":[0,0,0,0,0,0],"tolerance":1.0}`

---

## C. `is_moving` 自前判定（レポート 3.6）
変更: ファームの `is_moving()` フラグではなく、前回サンプルとの角度差で算出（status 呼び出し間の変化を反映）。

- [ ] 静止状態で `GET /robot/status` を2回 → `is_moving:false`（張り付かない）
- [ ] 移動指令直後に `GET /robot/status` を連続で叩く → 動いている間は `is_moving:true`、到達後は `false` に戻る

---

## D. 安全 — 関節リミットを公式値へ（レポート 4 / 公式ドキュメント）
変更: 公式値に統一 — J1±168 / J2±135 / J3±150 / J4±145 / **J5 -155〜+160** / J6±180。バリデーションと jog クランプで超過を防止。

- [ ] 境界内 OK: `PUT /joints/6/angle {"angle":180,"speed":20}` → 200／到達
- [ ] 公式リミット超過は拒否: `PUT /joints/2/angle {"angle":150}` → **400**（J2 は ±135）
- [ ] `PUT /joints/4/angle {"angle":146}` → **400**（J4 は ±145）
- [ ] J5 非対称の確認: `{"angle":160}` 200 / `{"angle":-160}` は 400（下限 -155）
- [ ] ⚠️ 重要: レポートで J4 が **-161.98°** まで動いた個体のため、**実機ファームが ±145 超を許すか**を要確認。
  - 実機が ±145 超を正当に必要とするなら「関節リミットを設定可能化」へ切替（要相談）

---

## E. エラーハンドリング（レポート 5 / 公式 `get_error_information`）
変更: firmware エラーコードを文章化し、移動系の 503 詳細に付与。`/robot/status?include_error=true` でも返却。

- [ ] `GET /robot/status?include_error=true`
  - 合否: 正常時 `error_code:null` / `error_message:null`
  - 合否: なんらかのフォルト発生時（例: リミット/衝突保護）に `error_code` と意味のある `error_message`（例 "Joint 4 limit exceeded" / "Collision protection triggered"）
- [ ] 既定（フラグなし）の `GET /robot/status` は従来どおり**即応答**（エラー読みで遅くならない）
- [ ] 失敗する移動を起こした際の 503 detail に `[robot error N: …]` が付く

---

## F. 起動時 `set_fresh_mode(1)`（公式 / レポート 3.1 緩和）
変更: 起動時に「最新指令優先」モードを設定し、stop・新目標の即時反映性を改善。

- [ ] 移動中に新しい目標を送ると、前の指令を待たずに**即座に**新目標へ向かう
- [ ] 移動中に `POST /robot/stop` → ほぼ即停止（積み残しで動き続けない）

## G. `stop()` の検証・再送
変更: firmware が未停止(0)を返したら数回再送して確実に停止。

- [ ] `POST /robot/stop` で確実に停止（A・B のタイムアウト/手動停止で実効性を確認）

## H. グリッパー状態取得（公式 `get_gripper_value` / `is_gripper_moving`）
変更: 新 `GET /gripper/status`（＋MCP `get_gripper_status`）で開度と移動中を取得。

- [ ] `curl -s 'http://192.168.0.136:8080/gripper/status?gripper_type=3'`
  - 合否: `{"value":0-100,"is_moving":bool,"timestamp":...}`
- [ ] `set_value` 実行直後は `is_moving:true`、完了後 `false`
- [ ] 不正タイプ `?gripper_type=2` → 400

---

## I. リグレッション（維持したいコア動作 / レポート §6）
- [ ] `get_robot_status` 即応答
- [ ] `get_joint_angles` 正常
- [ ] `move_joint`（J1→30° 到達）
- [ ] `move_all_joints`（6関節同時、目標±誤差内）
- [ ] `go_home`（0°復帰）
- [ ] `stop_robot` 即停止
- [ ] gripper open / close / calibrate / set_value 正常

---

## J. MCP 経由（Claude Desktop）スポット確認
REST が OK なら契約は満たすが、MCP ツールでも確認:

- [ ] `get_robot_status` … 角度・is_moving に加え `error_code` 文脈が見える
- [ ] `jog_joint`（`increment` 指定）… 即応答・約 increment° 動く
- [ ] `control_gripper` `release` … 正常（503 が出ない）
- [ ] `get_gripper_status` … 開度＋移動中
- [ ] `wait_for_movement`（`tolerance` 指定）… `reason`（converged/stalled/timeout）と `max_error` が返る

---

## メモ欄（実測値を残す）
- 到達誤差（move_all → 20° / go_home → 0°）:
- wait の `reason` と `max_error` 実測:
- 関節リミット実機挙動（特に J4 が ±145 超を許すか）:
- 要追従の不具合:

---

# 再テスト項目（2回目修正分）

1回目の実機テストで見つかった3件への対応。サーバー再起動（`./mycobot_server_ctl.sh restart`）と MCP 再ビルド（`cd mcp-server && npm run build`）後に実施。

## R-0. 事前調査（②③の前提）
- [ ] 実機側で `python3 scripts/introspect_pymycobot.py --port /dev/ttyACM0`
  - 記録: `pymycobot version` = ______
  - 記録: `get_angle`（単数）= OK/NO、`get_gripper_value` = OK/NO、`is_gripper_moving` = OK/NO
  - `get_gripper_value` が NO の場合、`dir()` 一覧から代替の開度取得メソッドが無いか確認

## R-1. jog の `get_angle` 修正（① / 前回 ❌）
変更: `get_joint_angle` を `get_angles()[joint-1]` に修正。
- [ ] `curl -s -X POST http://192.168.0.136:8080/joints/1/jog -d '{"direction":1,"speed":20,"increment":5}' -H 'Content-Type: application/json'`
  - 合否: **200**（`'MyCobot' object has no attribute 'get_angle'` が出ない）／J1 が約 +5°
- [ ] `GET /joints/1/angle` → 200 で現在角を返す（このバグも同時解消）
- [ ] MCP `jog_joint`（increment 指定）も 200・実際に動く

## R-2. gripper status の堅牢化（② / 前回 ❌）
変更: 未実装メソッドは **501 Not Implemented**＋明示メッセージで返す（503 の生エラー回避）。
- [ ] `curl -i -s 'http://192.168.0.136:8080/gripper/status?gripper_type=3'`
  - R-0 で `get_gripper_value` が **NO** なら: **501** ＋「not available in this pymycobot/firmware version」
  - R-0 で **OK** なら: **200** で `{value, is_moving}`（この場合は本来の取得が成功）
- [ ] 501 の場合、代替メソッドが判明すれば controller を実在 API に置換（要相談）

## R-3. go_home ハングの切り分け（③ / 前回 ❌）
コード上 `home_position == move_all_joints([0]*6)`。層を切り分ける。
- [ ] **REST 直叩き**（MCP を経由しない）:
  `time curl -s -X POST http://192.168.0.136:8080/robot/home -d '{"speed":25}' -H 'Content-Type: application/json'`
  - 即 200 ＆ ホーム復帰 → **python/REST は正常 → 原因は MCP/node 層**
  - ここでハング → **python/serial 層が原因**（次項へ）
- [ ] 直後に `curl -s -X PUT http://192.168.0.136:8080/joints/angles -d '{"angles":[0,0,0,0,0,0],"speed":25}' -H 'Content-Type: application/json'` と挙動を比較（差が出るか）
- [ ] 何度か繰り返し、**間欠性**を確認（毎回か/たまにか）

## R-4. MCP 側リクエストタイムアウト（4分ハング防止）
変更: MCP の HTTP クライアントに既定 **30秒** のタイムアウト（`MYCOBOT_API_TIMEOUT_MS` で上書き可）。
- [ ] go_home が万一詰まっても、MCP ツールは **30秒程度で**「Request timed out after 30000ms — the robot may be busy…」を返す（4分ハングしない）
- [ ] タイムアウト後 `stop_robot` → `get_robot_status` で復帰できる

## R-5. リグレッション（前回 ✅ の維持）
- [ ] release 200 / wait の converged・timeout＋自動停止 / 関節リミット 400 / fresh_mode 即反転 / is_moving 非張り付き

## 再テストのメモ
- introspect 結果（version / 実在メソッド）:
- go_home: REST 直叩きの可否（=層の切り分け結果）:
- MCP タイムアウト発火の確認:
