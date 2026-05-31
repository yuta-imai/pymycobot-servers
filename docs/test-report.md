# mycobot MCP サーバー 改善ポイントまとめ

実機（MyCobot, REST API `http://192.168.0.136:8080` 経由）に対して全ツールを一通り検証した結果と、改善提案をまとめたもの。

- 検証日: 2026-05-31
- 構成: Claude Desktop (Windows) → `wsl` → node (nodenv shims) → MCP サーバー → REST API → MyCobot 実機
- サーバー: `mycobot-mcp-server` v0.1.0 / REST API v1.0.0

---

## 1. 検証結果サマリ

| 機能 | 結果 | 備考 |
|------|------|------|
| `get_robot_status` | ✅ | 安定。即応答 |
| `get_joint_angles` | ✅ | 正常 |
| `move_joint` | ✅ | J1→30° 正確に到達 |
| `move_all_joints` | ✅ | 6関節同時、目標±0.3°で到達 |
| `go_home` | ✅ | 0°復帰OK |
| `stop_robot` | ✅ | 即停止 |
| `wait_for_movement` | △ | 動作するが完了判定が不正確 |
| `jog_joint` | ❌ | 約4分応答なし・ハング・実際に動かない |
| gripper: open / close / calibrate / set_value | ✅ | 4種正常 |
| gripper: release | ❌ | flag 値バグ（503） |

コア動作（status / 単関節・全関節移動 / 主要グリッパー操作）は安定。修正対象は明確なバグ2件＋判定ロジック・安全性の改善。

---

## 2. 明確なバグ

### 2.1 `jog_joint` がハングする（最優先）

- 呼び出すと応答が返らず、約4分でクライアント側タイムアウト。
- 呼び出し後も対象関節の角度は変化なし（J1 が 19.77° のまま）→ jog は実行されていない。
- 直後の `get_robot_status` は即応答 → **サーバー全体は生きており、jog_joint ハンドラ内でブロッキングが発生**している。
- 原因の見当: pymycobot の連続ジョグ系 API を呼んだまま、停止/完了待ちでブロックしている可能性が高い。
- 対策: 非ブロッキング化（増分移動して即リターン、または内部タイムアウト）。後述の `wait_for_completion` ロジックに乗せ替えるのが理想。

### 2.2 `release`（グリッパー）の flag 値バグ

エラー内容:

```
REST API 503 Service Unavailable: Failed to release gripper:
The data supported by parameter flag is 0 or 1 or 254, but the received value is 10
```

- release ハンドラが pymycobot のグリッパー解放（トルクオフ）に渡す flag に `10` を渡している。
- 仕様上 flag は `0 / 1 / 254` のみ。トルクオフは通常 `254`。
- open=0 / close=1 は正しく動作しているので、**release だけ定数マッピングを取りこぼしている**典型パターン。
- 対策: release 時の flag を `254`（実機仕様に合わせる）へ修正。

---

## 3. 完了判定ロジックの改善（`is_moving` / `wait_for_movement`）

### 3.1 現状の問題

- 目標到達後も `is_moving: true` が残る。
- `wait_for_movement` が、小さな移動でも `completed: false`（タイムアウト）を返す。
- go_home・move_all_joints・ホーム復帰のすべてで再現。
- 現状はファーム由来フラグや単純タイマーに依存していると推測。
- 加えて、タイムアウトで wait が返っても **実機は動き続ける**（go_home が止まらないのを実測）。

### 3.2 設計方針：終了条件を3つに分ける

| # | 条件 | 判定 | アクション |
|---|------|------|-----------|
| 1 | 収束（成功） | 全関節が目標角に対し許容誤差内 | 完了を返す（stop 不要） |
| 2 | ストール（失敗） | 未収束のまま一定時間ほぼ動かない | stop してから返す |
| 3 | タイムアウト（失敗） | 規定時間を超過 | stop してから返す |

- **1（許容誤差）** … サーボ精度起因の「終わらないコマンド」を防止。
- **3（タイムアウト＋キャンセル）** … 上限で必ず終わらせ、かつ実機を止める。
- **2（ストール検知）** … 物理的に届かない場合（例: 関節が引っかかる）にハングしないため。1 と 3 だけでは拾いきれないケースを補完。

### 3.3 パラメータの根拠（実測ベース）

到達誤差の実測:

- `move_all_joints → 20°`: 実測 19.68〜20.3°（誤差 ±0.3°程度）
- `go_home → 0°`: 実測 -0.17〜0.35°

→ **許容誤差 `tol = 1.0°`** 程度で、定常誤差・ディザリングを飲み込みつつ、明らかな未到達は弾ける。

### 3.4 ストールとスローの区別

低速（speed 5）の正当な移動と、引っかかりによるストールは瞬間速度では区別しづらい。
**「直近 T 秒の正味移動量」**で判定すると自然に分離できる。

- 直近の窓（例 2.5 秒）で目標方向に一定量（例 0.3°）以上進んでいれば「遅いが前進中」→ 継続
- 窓内の正味移動が閾値未満かつ未収束 → ストール

（J4 のケースは「数分で約1°」だったため、この基準で明確にストール判定される）

### 3.5 実装イメージ

ポーリングは HTTP 往復のない REST / Python 層（pymycobot に近い側）に置く。
MCP(node) 側から毎回 get_angles を叩くと往復コストが乗るため、
`POST /wait_for_completion`（tolerance, timeout を受ける）として Python 側に閉じ込めるのがよい。

```python
def wait_for_completion(target, *, tol=1.0, timeout=15.0,
                        poll=0.1, stall_window=2.5, stall_min_progress=0.3):
    start = time.monotonic()
    history = deque()  # (t, angles)
    while True:
        now = time.monotonic()
        cur = mc.get_angles()  # 取得失敗時の None ガードも要る
        err = max(abs(c - t) for c, t in zip(cur, target))

        # 1) 収束 → 成功
        if err <= tol:
            return {"completed": True, "reason": "converged",
                    "elapsed": now - start, "max_error": err}

        # 2) ストール検知（窓内の正味移動で判定）
        history.append((now, cur))
        while history and now - history[0][0] > stall_window:
            history.popleft()
        if now - start > stall_window and len(history) >= 2:
            progress = max(abs(c - h) for c, h
                           in zip(cur, history[0][1]))
            if progress < stall_min_progress:
                mc.stop()
                return {"completed": False, "reason": "stalled",
                        "elapsed": now - start, "max_error": err}

        # 3) タイムアウト → キャンセル
        if now - start > timeout:
            mc.stop()
            return {"completed": False, "reason": "timeout",
                    "elapsed": now - start, "max_error": err}

        time.sleep(poll)
```

### 3.6 注意点

- **target をどう知るか**: 現状の wait は引数なし。直前コマンドの目標をサーバー側で保持するか、wait に target を渡す。単腕なら「最後の指令目標を保持（last-write-wins）」で十分。
- **stall 閾値はディザリングの上に置く**: 静止時ノイズは ±0.5°未満だったので、窓 2.5 秒で `stall_min_progress=0.3` 程度。実機で要微調整。
- **収束時は stop しない**: 既に止まっているので不要。stop するのは 2 / 3 のときだけ。
- **`is_moving` を自前計算に置き換える案**: ファーム由来フラグが張り付くため、「前回ポーリングから eps 以上動いたか」で自前判定すれば status の信頼性も上がる。
- **動的タイムアウト**: 大きな移動＋低速は正当に時間がかかるため、固定値より「移動量と速度から見積もった値＋上限」にするとより堅い（初期は固定でも可）。
- **返り値を理由付きに**: `{completed, reason, elapsed, max_error}` とすると、クライアント側で「収束／ストール／時間切れ」を区別して扱える。

---

## 4. 安全ガード

- 速度5での go_home 中、J4 がホーム（0°）とは逆方向に動いて **-161.98°**（限界 -165° の手前）まで張り付いた。リセットで解消したため主因は実機側だが、サーバー側でも保険を入れたい。
  - (a) 関節が指令と逆方向／不動のときに検知して中断（= 3.2 のストール検知で兼ねられる）。
  - (b) 安全限界手前でのクランプ（指令値・動作中ともに）。

---

## 5. エラーハンドリングと入力検証

- release の 503 のように、下位ライブラリの生エラーがそのまま透過している。
  - (a) 引数を送る前にバリデーション（flag・値域チェック）。
  - (b) 503 等を意味のあるメッセージに整形して返す。
- → MCP クライアント側での原因特定が速くなる。

---

## 6. 良かった点（維持したい）

- node 解決（nodenv shims のフルパス指定）が確実。
- `MYCOBOT_API_BASE_URL` を起動コマンドにインライン指定して、WSL 内プロセスへ確実に渡している。
- 起動ログを **stderr** に出している（stdout を汚さず、MCP の stdio 規約に適合）。
- コア動作（status / get_joint_angles / move_joint / move_all_joints / go_home / stop / 主要グリッパー操作）は安定。

---

## 7. 対応の優先度

1. `jog_joint` のハング解消（非ブロッキング化）
2. `release` の flag 値を `254` に修正
3. `wait_for_completion` 導入（許容誤差＋ストール＋タイムアウト＋自動 stop）
4. `is_moving` の自前判定化／安全限界クランプ
5. 入力バリデーションとエラーメッセージ整形


# mycobot MCP/REST 改善 — 実機テスト結果

[test-checklist.md] に沿って、MCP ツール経由（Claude Desktop → WSL → REST API → 実機）で検証した結果。

- 検証日: 2026-05-31
- 対象: REST API `http://192.168.0.136:8080`
- サーバー: `mycobot-mcp-server` / REST API v1.0.0
- グリッパー: parallel（`gripper_type=3`）
- 速度: 低速（10〜30）で実施
- 注記: 実機 REST へは MCP ツール経由で到達。純粋な `curl`/REST 限定項目は、対応する MCP ツールで実質的に検証。

---

## 結果サマリ

| 項目 | 結果 | 備考 |
|------|------|------|
| §0 準備 | ✅ | healthy / `error_code: null` / 静止時 `is_moving: false` |
| §A-1 jog ハング解消 | ✅ | ハング解消。即エラー応答 |
| §A-1 jog 動作 | ❌ | 新バグ `'MyCobot' object has no attribute 'get_angle'`（503） |
| §A-2 release flag | ✅ | 200 成功、503 解消。open/close リグレッションなし |
| §B 収束判定 | ✅ | `converged` / max_error 0.26〜0.7 ≤ tol |
| §B タイムアウト＋自動停止 | ✅ | `reason:timeout`（max_error 57.16）後、実機停止を確認 |
| §C `is_moving` 自前判定 | ✅ | 静止で false、張り付かない |
| §D 関節リミット | ✅ | J2/J4/J5 すべて 400 拒否。J5 非対称も正しい |
| §E エラー情報 | ✅(部分) | フィールド追加・正常時 null（フォルト誘発は未実施） |
| §F 最新指令優先 | ✅ | 指令中に逆方向指令を送ると即反転 |
| §G stop | ✅ | 即停止・位置保持 |
| §H グリッパー状態取得 | ❌ | `'MyCobot' object has no attribute 'get_gripper_value'`（503） |
| §I リグレッション | ✅ | コア動作すべて正常 |
| **go_home** | ❌ | 4分ハング・無動作（新規/間欠） |

---

## 詳細（チェック項目別）

### §0. 準備
- [x] `GET /robot/status` → `healthy` / `robot_connected: true`
- [x] `error_code: null` / `error_message: null`（フィールド追加を確認）
- [x] 静止時 `is_moving: false`

### A-1. `jog_joint` のハング解消
- [x] **ハング解消**: 呼び出しは即座に応答（4分タイムアウトは発生しなくなった）
- [ ] ❌ **動作せず**: `direction:1, increment:5` で 503
  `Failed to jog joint 1 (direction=1, increment=5.0): 'MyCobot' object has no attribute 'get_angle'`
  - 現在角取得に存在しない `get_angle`（単数）を使用。`get_angles()[joint-1]` に修正が必要。
  - jog 実行後も J1 は -0.17° のまま（未実行）。
- [ ] increment 範囲外 422 / リミットクランプ → get_angle バグでブロックされ未検証

### A-2. `release`（グリッパー）の flag バグ
- [x] `release`（`gripper_type:3`）→ **200**。以前の 503 "flag … received 10" は解消。
- [x] `open` / `close` 正常（リグレッションなし）

### B. 完了判定 `wait_for_movement`
- [x] 収束: `move_all_joints [20,0,0,0,0,0] speed30` → wait(tol=1.0) が
  `completed:true / reason:"converged" / max_error:0.26`
- [x] タイムアウト＋自動停止: `move_all_joints [-120,...] speed10` → wait(timeout=1.0) が
  `completed:false / reason:"timeout" / max_error:57.16`。
  **その後ロボットが停止**（J1 = -63.8° のまま約1分静止を確認）。以前の「動き続ける」挙動は解消。
- [ ] `stalled`: 安全に物理ストールを再現できず未検証
- [~] target 明示指定: MCP ツールは target 引数を公開していないため未確認（last-write-wins 前提で動作）

### C. `is_moving` 自前判定
- [x] 静止時 `is_moving:false`（張り付かない）
- [x] 急停止直後に一瞬 `true`（実際の整定微動 0.1°程度を差分検知）→ 静止後 `false`。実挙動を反映。

### D. 関節リミット（公式値）
- [x] J2→150° → **400** `Valid range: -135 to 135`
- [x] J4→146° → **400** `Valid range: -145 to 145`
- [x] J5→-160° → **400** `Valid range: -155 to 160`（非対称リミットを確認）
- [~] ⚠️ 実機ファームが ±145 超を許すかは、サーバーが送信前に弾くため**確認不可**（=サーバー側ガードは有効）

### E. エラーハンドリング
- [x] `GET /robot/status` に `error_code` / `error_message` が追加され、正常時は null
- [x] 既定の status は従来どおり即応答（エラー読みで遅延しない）
- [ ] フォルト時の文章化メッセージ → フォルト未誘発のため未検証

### F. 起動時 `set_fresh_mode(1)`
- [x] 移動中に新目標を送ると前の指令を待たず即反映（+60°指令中に-120°を送り、即反転して-120°到達）

### G. `stop()`
- [x] 移動中に `stop_robot` → 即停止、位置保持（J1=-52.73° で停止し目標へ進まず）

### H. グリッパー状態取得（新規）
- [ ] ❌ `get_gripper_status`（`gripper_type:3`）→ 503
  `Failed to get gripper status: 'MyCobot' object has no attribute 'get_gripper_value'`
  - 存在しない `get_gripper_value` を使用。実機 pymycobot の実在メソッドへ要置換。
- [ ] `set_value` 後の is_moving / 不正タイプ 400 → 上記バグでブロックされ未検証

### I. リグレッション
- [x] `get_robot_status` 即応答
- [x] `get_joint_angles` 正常
- [x] `move_joint`（J1→30° 到達は前回確認）
- [x] `move_all_joints`（6関節同時、目標±誤差内）
- [x] `stop_robot` 即停止
- [x] gripper open / close 正常
- [~] `go_home` → 下記の通り NG

### 追加観測: `go_home` ハング（新規/最優先）
- `stop_robot` 後に `go_home(speed=25)` を呼ぶと **4分応答なし・J1 は無動作**。
- サーバーは生存（直後の `get_robot_status` は即応答）。
- 同状態から `move_all_joints [0,0,0,0,0,0]` は**即成功**でホーム復帰 → ハングは **go_home 固有**。
- go_home 内部の完了待ち or 特定 API 呼び出しがブロックしている疑い。§B のタイムアウト/自動停止が go_home 経路に組み込まれていない可能性。

---

## 要対応（優先度順）

1. **`go_home` のハング解消**（最優先）。move 系は正常なので go_home 経路の見直し。タイムアウト/自動停止ロジックを go_home にも適用。
2. **pymycobot メソッド名の取り違え 2件**
   - `jog_joint`: `get_angle` → `get_angles()[joint-1]`
   - `get_gripper_status`: `get_gripper_value` が実機 pymycobot に存在しない。バージョン確認のうえ実在 API へ置換。
   - 根本原因の疑い: 新規コードが別バージョン/別機種（Pro/320 系）の pymycobot API 前提。`pymycobot.__version__` と `dir(mc)` で実在メソッドを確認すること。

## 達成済み（前回 ❌/△ からの改善）
- `release` の flag バグ解消（503 → 200）
- `jog_joint` のハング解消（ただし別バグ残）
- `wait_for_movement` の許容誤差収束・タイムアウト自動停止・`reason`/`max_error` 返却
- `is_moving` の張り付き解消（自前判定）
- 関節リミットの公式値バリデーション（非対称 J5 含む）
- 最新指令優先（fresh_mode）と stop の実効性

---

## 実測メモ
- 到達誤差（wait の max_error）: move_all→20° で **0.26°** / →0° で **0.70°**
- wait の `reason`/`max_error` 実測: `converged`(0.26, 0.70) / `timeout`(57.16)
- 関節リミット拒否メッセージ: J2 `-135〜135` / J4 `-145〜145` / J5 `-155〜160`（非対称）
- 関節リミット実機挙動（J4 が ±145 超を許すか）: **未確認**（サーバーが送信前に 400 で弾くため）
- 要追従の不具合: ① go_home ハング ② jog の get_angle ③ gripper status の get_gripper_value
