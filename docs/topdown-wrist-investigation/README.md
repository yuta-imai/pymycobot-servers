# Top-down 把持が真下を向かない問題 — 調査ログ & 再開メモ

最終更新: 2026-06-02（検証・診断完了 / 修正は未着手）
ブランチ: `feat-integration` / 対象実機: MyCobot 280 M5（ホスト名 `mycobot`, Raspberry Pi）

このファイルだけ読めば議論を再開できるようにまとめてある。関連する永続メモ:
`~/.claude/projects/-home-factory-playground-pymycobot-servers/memory/topdown-ik-wrong-wrist-model.md`

---

## 0. 目的（このセッションでやったこと）

「Home で soracam 基準画像を撮り、それを基準に `pick_at`（＝真下把持）でグリッパーが
正しく真下を向くか」を実機で動作確認したい、という依頼。

- `pick_at` は**未実装**（[../integration-plan.md](../integration-plan.md) の A2 として設計のみ）。
  REST にも top-down/pick エンドポイントは無い。真下把持の実体は
  [`mycobot_joint_controller.py`](../../mycobot_joint_controller.py) の
  `solve_topdown_ik(x,y,z)`（`orientation_mode="X"` で tool-X 軸を真下に拘束）。
- よって「`solve_topdown_ik` が実機で真下を向くか」を検証した。

## 結論（先に）

1. **`solve_topdown_ik` は実機で真下を向かない。約45°オーバーハングする。**
2. 原因は **公称キネマティクスモデル（URDF＝firmware内部モデル＝公式DH）の「手首」が、
   この個体の実際の手首と一致しない**こと。
3. **URDF ファイルそのものは本物の公式版**（上流とバイト一致・破損なし）。コードの関節順序
   バグでもない。＝ ファイル編集ミスではなく「公式モデルの記述 ⇄ 実機ハードの実差」。
4. **位置FKは正確（~2–10mm）。姿勢FKだけが手首で壊れている。**
5. **副次的に重要**: firmware `get_coords` の**姿勢(rx,ry,rz)も信用できない**
   （モデルと同じ手首誤差を共有する。位置は信頼可）。
6. 手首DHパラメータの再設定**だけ**では「位置」と「姿勢」を両立できない（後述の探索で確認）。
   → 根治には**きちんとした手首較正**が必要。

---

## 1. 環境・アクセス情報（再開時に必要）

- **dev box（このリポジトリの作業機, WSL2）**: 実機シリアルは無い。だが
  `.venv` に numpy 2.4.6 / ikpy 3.4.2 / pymycobot 4.0.4 を導入済みなので、
  **URDF/DH のオフライン FK 比較は実機なしでローカル完結できる**:
  `.venv/bin/python ...`
- **実機ホスト**: `ssh -i ~/.ssh/id_mycobot factory@192.168.0.136`、リポジトリは `~/arms`、
  同じ git ブランチ（dev box で push → 実機で pull で同期）。
- **実機の駆動**: REST API サーバ(8080)は落ちていた。シリアル競合を避けるため
  **`~/arms/.venv/bin/python3` で controller を直接叩いて `/dev/ttyACM0` を駆動**した。
  例: `MyCobotJointController("/dev/ttyACM0",115200)` → `mc.send_angles(...)`。
  - 安全Home: `send_angles([0,0,0,0,0,0], 25)`（firmware `go_home` はハングするので使わない）。
- **soracam**: MCP ツール `mcp__soracam__get_live_still_image`（dev box から呼ぶ）。
  返り値が大きく token 上限を超えるので、保存された tool-results txt から base64 を
  デコードして JPEG 化 → 画像として確認した。撮影に約15秒。カメラは**非常に広角**で、
  横移動が奥行き風の歪みに見えることがある（ユーザー指摘）。

## 2. カメラ ⇄ base 座標の対応（ユーザー目視で確定）

- **base +X（アームのリーチ方向 / 前）= カメラの「手前」方向**
- **base +Y（左）= 画面内の横方向**

---

## 3. 実機スイープ実測（canonical `[0,0,-90,0,90,0]` 起点, soracam＋ユーザー目視）

| スイープ（指令角） | モデル(URDF=firmware)予測 | 実機の実測（確定） | 判定 | 画像 |
|---|---|---|---|---|
| `[0,0,0,0,0,0]` Home | — | 腕が真上, rpy≈[-90,0,-90] | 基準 | `img/00_home_baseline.jpg` |
| `[0,0,-90,0,90,0]` canonical | 真下 | 真下（ペン確認とも一致） | OK | `img/01_canonical_down.jpg` |
| `[0,0,-90,0,90,90]` J6=+90 | 接近軸90°傾く | **腕構造不変・グリッパー自転のみ**（指先とJ6が同軸） | モデル誤り | `img/02_j6_90.jpg` |
| `[0,0,-90,-40,90,0]` J4=-40 | X(奥行き)平面に40° | **+Y(横)に40°** | 平面90°ずれ | `img/03_j4_-40.jpg` |
| `[0,0,-90,0,60,0]` J5=+60 | -Y(横)に30° | **+X(前/奥行き)に30°** | 平面90°ずれ | `img/04_j5_60.jpg`, `05_j5_60_replay.jpg` |

実測の傾き量は関節角と 1:1（J4=-40→40°, J5 -30→30°）。J6 はグリッパー指先と同軸の自転。

### top-down 解での照合（target (180,0,150)）
- `solve_topdown_ik(180,0,150)` → `[6.1,-18.2,-97.6,-11.7,90.0,-37.5]`、実機実行。
- 実機到達位置は目標どおり（~10mm 以内, real coords `[178.9,2.7,139.7]`）。
- だが**目視でグリッパーは約45°オーバーハング**（ユーザー観測）。
- 仕組み: モデルは J6=-37° の「傾き」で接近軸を 40.4°→3.3°(≈真下) に曲げ戻し、内部の
  downness 自己チェック(0.998)を通している。実機の J6 は傾けない（自転のみ）ので
  この補正が起きず、残留傾斜 ≈ 45°。**downness 自己チェックは壊れたモデルでの自己採点**。

---

## 4. オフライン診断（dev box `.venv`, 実機不要で再現可能）

すべて `MyCobotJointController` をオフライン生成（`__new__` で serial を張らず `_init_ik_chain()` のみ）
してURDF FKを評価。DHは [`../../scripts/verify_dh.py`](../../scripts/verify_dh.py) のテーブル。

1. **URDF FK と DH FK はほぼ一致**（位置~2mm）。両方とも実機スイープ期待と食い違う。
2. **URDF FK 姿勢 と firmware `get_coords` 姿勢は <0.5° で一致**（home も top-down 解も）。
   → URDF はロボットの公称キネマティクスを忠実に再現。＝「壊れたURDF」ではない。
   → だが両者そろって実物グリッパーとはズレる（同じモデルを共有しているから）。
3. **canonical での各手首関節の world 回転軸**:

   | 関節 | モデルの軸 | 実機の軸（実測） |
   |---|---|---|
   | J4 | -Y | **+X** |
   | J5 | +X | **-Y** |
   | J6 | +Y | **-Z（真下=接近軸）** |

   実機 = 直交した綺麗な3軸(X,-Y,-Z)。モデルは **J4とJ6が両方±Y で平行＝canonical が手首特異点**。
   実機は同じ指令角で非特異。**モデルが実機の非特異姿勢で特異**になっている。
4. **実機接近軸はモデルのツール座標系でも一定でない**（canonical/J4/J5 で最大32°ばらつき）
   → 一定オフセットや接近軸の付け替えだけでは直らない。

### コードの確認
- IK チェーンが使うのは**バレ URDF のみ**: `_URDF_PATH = urdf/mycobot_280_m5.urdf`
  （[`mycobot_joint_controller.py:47`](../../mycobot_joint_controller.py)）。gripper 版URDFは未使用。
- 関節→ikpyスロット対応は**URDFチェーンの自然順を保持**（`ik_active_idx`）＝J4/J5の誤順序なし。
- URDF の git 履歴: `c696f21 "new ik strategy"` で**新規追加のみ**。reflog にも編集→revert 痕なし。
- 記憶にあった「URDF修正」の実体は**コード側**（`e5ce8ef` が接近軸を Z→X に変更、
  `0822e51` は元々 `orientation_mode=Z`）。URDF ファイルは無罪。

---

## 5. 修正の試み（オフライン）と、なぜ簡単に直らないか

`scripts/verify_dh.py` の DH を出発点に、手首3関節(J4/J5/J6)の alpha と theta-offset を補正して
実測4条件（canonical真下 / J6 roll / J4→+Y / J5→+X）＋位置を再現できるか試した。

- **least_squares（小補正）**: 局所解に落ち、接近軸誤差 50–86° で停滞。
- **±90°級の離散構造探索**: 最良解でも canonical真下とJ6 rollは合うが、J4/J5 の傾き平面が
  外れ(54°/41°)、しかも**位置が ~62mm 破綻**。

要点:
- **公式DH（無変更）→ 位置は良好だが姿勢が誤り。手首DHを変えて姿勢を直すと位置が破綻。**
  → 手首DHパラメータの再設定**だけ**では位置と姿勢を両立できない。
- 加えて実測の傾き方向(+X/+Y)は**目視＋未較正カメラ由来で符号が不確か**。位置は符号と無関係に
  破綻するので、**この粗い目視3点では正しいモデルを同定できない**。

> 注: `verify_dh.py` の docstring「patching one origin breaks J6 downstream」と整合。

---

## 6. 根治＝きちんとした手首較正（汎化方針 / 次のステップ）

本番・デモ移設でも同じ手順で再較正できることを目標にする。

1. **基準系の確立（定量化が肝）**: カメラ⇄base 外部パラメータ（ArUco ハンドアイ）を取るか、
   簡便には**分度器/治具でグリッパー接近軸の実角度を定量測定**できるようにする。目視脱却。
2. **較正データ収集**: 手首3関節を複数角度 × 複数の上流姿勢（J1≠0, J4+J5複合）で振り、
   各姿勢の接近軸方向を定量記録（数十点）。
3. **モデル同定**: 位置（既に正確）を拘束しつつ姿勢を再現する補正キネマティクスを最小二乗
   フィット＋**全データ交差検証**。
4. **組込み＆実機再検証**: `solve_ik`/`solve_topdown_ik` を補正モデルに差し替え（安全ゲートは
   維持）、本ログのスイープ＋top-down で再検証。
5. [../integration-plan.md](../integration-plan.md) 末尾「較正手順書」TODO と一致。

### 採用方針: エンド側に加速度センサー（2026-06-02 決定）
soracam 目視の代わりに、**グリッパー（フランジ）に剛固定した加速度センサーで重力ベクトルを
定量測定**し較正する。静止時 `g_sensor = R_mount^T·R_flange(q)^T·g_world`。接近軸の鉛直からの
傾きを 1°未満で直接測れる。原理的限界＝**鉛直まわりヨーは不可観測**（＝J6自転、把持時に開放
する分なので傾き較正には無害）。
- センサー: **WitMotion WT9011DCL（BLE5.0 9軸, 9g, 40h電池, roll/pitch精度0.2°）**を採用。
  Pi が BLE で読む。**MyCobot の M5 FW には触れない**（だから M5 IMU は不採用）。
  - プロトコル（実装済み）: service `FFE5`／notify `FFE4`。`0x55 0x61` の20バイト結合パケット
    （accel/gyro/angle が int16LE, 加速度=raw/32768×16g, 角速度=raw/32768×2000°/s）。
  - **生の加速度（重力ベクトル, roll/pitch 0.2°）を使用**。ヨー(Z,磁気依存1°)は使わない
    （モーター/金属の磁気乱れ回避＋本手法はヨー不可観測前提）。角速度は静止判定に流用可。
  - 取付は**フランジに剛固定**（`R_mount` 一定が命）。較正中は動かさない。

### 実装済み: 較正フレームワーク `scripts/wrist_calib/`（option (b) を実装）
ハード非依存・オフライン検証済み。BLE 実装だけ品番確定後に差し込む。
- `gravity_source.py` — `GravitySource` 抽象 + `Manual`/`Replay` + **`BLEGravitySource`**。
  **WT9011DCL の WitMotion パーサ実装済み・テスト済み**（結合/単フレーム両対応・通知分割を
  またぐバッファリング・ジャイロも抽出）。`bleak` 使用（Pi: `pip install bleak`）。nus/beacon は
  別センサー用スタブとして残置。
- `poses.py` — 較正姿勢生成（単関節スイープ/J4×J5格子/J1・腕ピッチ変種）。`default_set`=78姿勢。
- `collect.py` — **実機ホストで実行**。各姿勢で move→静止→関節角・coords・安定重力を記録し JSONL 出力。
  `--dry-run` でセンサ＋姿勢列を実機なしで素振り可。firmware send_coords/go_home は不使用。
- `calibrate.py` — オフラインで補正キネマティクス（DH補正＋`R_mount`＋base傾き）を最小二乗フィット＋
  k-fold 交差検証。`--selftest` は合成データで自己回復を確認（姿勢誤差 22.75°→0.05°）。
- 既知の縮退: 加速度のみでは J6 オフセットと `R_mount` の接近軸まわり成分が分離不能（＝ヨー不可観測）。
  合計の予測重力は一致するので傾き較正に影響なし。必要ならジャイロ/磁気で後日分離可能。

### RPi セットアップ（2026-06-03 検証済み）
- Pi: BlueZ 5.66 / hci0 UP、`~/arms/.venv` に `bleak 3.0.2`+`dbus-fast` 導入済み。
- WT9011DCL: **アドレス `F7:50:70:EE:0D:DF`（広告名 `WT901BLE67`）。接続はアドレス指定が確実。**
- 疎通確認OK: `python3 scripts/wrist_calib/collect.py --provider ble \
  --ble-address F7:50:70:EE:0D:DF --pose-set quick --dry-run`
  → 静置で `g≈[0.107,0.006,0.994] still=True motion=0.33dps`。生バイトでパーサ検証済み。

### 次の具体ステップ
1. （済）RPi/BLE セットアップ・疎通確認。
2. `collect.py --provider ble --pose-set default --out calib.jsonl` でデータ収集（実機, 78姿勢）。
3. `calibrate.py --data calib.jsonl` でフィット＆CV → 残差が小さければ採用。
4. 補正モデルを `solve_ik`/`solve_topdown_ik` に組込み（接近軸＝J6軸を真下拘束・J6はヨー開放、
   現行 `orientation_mode="X"` を置換）。安全ゲート維持のまま本日のスイープ＋top-down で再検証。

---

## 7. 重要な未解決点 / 仮説

- 手首DH単独で位置・姿勢を両立できない事実は、ズレが**手首DHパラメータ以外**（角度規約の
  写像、または手首より上流のフレーム、あるいは個体差）にもある可能性を示す。較正データが
  揃えば切り分け可能。
- 真の接近軸 = J6 回転軸（指先と同軸）。較正後の IK は「J6軸を真下に拘束、J6はヨー用に開放」
  という定式化が素直（現行の `orientation_mode="X"`＝tool-X拘束は誤り）。

## 8. 安全メモ
- firmware `send_coords`（Cartesian）は壊れたIKで暴走するので**呼ばない**（J3ハードリミット
  突入の前科あり）。Cartesianは必ず自前IK→`send_angles`。
- firmware `go_home` はハング（~4分）。Homeは `send_angles([0,0,0,0,0,0])`。
- 実機を動かす検証は周囲をクリアにし、send_angles 後に `get_coords`/角度で着地確認。
