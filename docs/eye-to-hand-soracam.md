# Eye-to-Hand 視覚ピック — 単眼 SORACAM + A4 ChArUco（ホモグラフィ方式）

D435（深度カメラ）を使わず、**固定した単眼 SORACAM 1台 + 既知平面（A4 ChArUco）**で
トップダウン・ピックの座標を出す。対象は「ある程度大きい物体・ボード上に置く」、セット
アップ後は静的、という確定条件。

実装は `scripts/vision/`。純幾何（`geometry.py` / `placement_calculator.py` /
`touch_calibrate.py --simulate`）は numpy/scipy のみで自己テスト済み。OpenCV 依存部分
（`charuco_board` / `frame_source` / `homography_calibrate` / `locate`）と実機実画像でも
検証済み（後述）。

> 最終目標: 校正ロジックを固めたら **API / MCP サーバー経由**で実行できるようにする。
> そのため校正は対話の少ない**単発処理**で組み、各段を import 可能な関数に分けてある
> （`homography_calibrate.fit_from_frame` / `touch_calibrate.collect` / `locate.locate`）。

---

## なぜこの方式か

- **深度不要**: 対象が既知平面（ボード面）上 → 画素を平面に写すだけで XY が出る。配置
  解析（§2）から、適切な配置なら視覚誤差はアーム実行誤差（~3.5mm）より小さく、視覚は
  律速にならない。D435 を入れてもアームが精度を使い切れない。
- **内部校正すら不要（ホモグラフィ方式）**: 固定カメラ＋平面＋対象はボード内、という構成
  では、**1枚のボード撮影から「画素↔ボード平面mm」の平面ホモグラフィ**を当てれば十分。
  レンズ歪みは**ボード領域内の内挿として吸収**される。多視点の内部校正が要らず、API/MCP
  化に最適。実フレームで **leave-one-out 残差 mean 0.53 / p95 1.30 / max 1.47 mm**（square
  28mm 時、22コーナー）を確認済み。
- **soracam の読み取り性**: MCP `get_live_view_unlimited` は DASH URL のみ（エージェントは
  直接フレームを読めない）。`get_live_still_image` は base64 JPEG（デコードすれば可読・
  ~15s/枚・1920×1080）。production の連続フレームは **Pi 側 OpenCV が RTSP/DASH を直接食う**。
- **オープンループ専用**: クラウドカメラは同期/低遅延でないため「一度見て→計画→実行」。

---

## 物理ボード（実測で確定済み, 2026-06-07）

| 項目 | 値 |
|---|---|
| グリッド | **10 × 7 マス**（候補総当りで一意特定。コーナーが交点に正確に乗る） |
| 辞書 | **DICT_6X6_250**（35マーカー, id 0–34） |
| マーカー/マス比 | ~0.46（白枠が広い実ボード特性） |
| 用紙 | A4 |
| `square_length_mm` | **要実測**（メトリックスケールの基準。`config.BOARD` に設定） |
| フレーム解像度 | 1920 × 1080 |

`square_length_mm` が間違うとスケール誤差が残る（board→base の剛体フィットは吸収しない）。
**実測必須**。安全網として `touch_calibrate.py --scale` で相似スケール推定も可能（§B）。

---

## 座標チェーン

```
物体画素 ──H (画素→ボードmm)──▶ ボード(X,Y) ──board→base(タッチ校正)──▶ ベース(X,Y,Z)
                                                                          │
                                                       solve_topdown_ik ─▶ send_angles
```

- **H（画素↔ボード平面）** = `homography_calibrate`（単発・1枚）。
- **board→base** = ChArUco 既知コーナーを**先端でタッチ**し `corrected_fk`（accel校正済の
  信頼FK; firmware get_coords は使わない）で先端ベース座標を読み、Umeyama/Kabsch で当てる。
- **核心の相殺性質**: `solve_topdown_ik` の一致判定も同じ `corrected_fk` を使うため、FK の
  剛体バイアスは board→base に畳み込まれ**ピック時に相殺**する。firmware FK/IK が壊れた本機
  に最適。

---

## ワークフロー（3 段）

### ① 物理セットアップ
- アーム固定、SORACAM 固定（**真上寄り・近く**推奨＝§2）。A4 ChArUco を作業位置に平置きし、
  **作業面として常設**。1マスを実測 → `config.BOARD.square_length_mm` に反映。

### ② キャリブレーション（セットアップ後 1 回）
- **A: ホモグラフィ** `homography_calibrate.py --grab`（**アーム退避**でボード全面が見える1枚）。
- **B: board→base** `touch_calibrate.py --collect --n 6`（既知コーナー6点タッチ; §1）。
  square を実測していなければ `--scale` でスケール推定を併用。

### ③ ピック（毎回）
- 一度だけ空ボード参照 `locate.py --save-ref --grab`。
- 物体を置いて `locate.py --grab --json` → `base_xyz_mm` 等（`docs/integration-plan.md` の
  `locate` 契約に整合）。そのまま `solve_topdown_ik` / `pick_at` へ。ドリフトが気になれば
  `--refresh`（その場でHを取り直す）。

---

## ホモグラフィ方式の制約（承知の上で採用）

1. **1平面のみ**: 正しい3Dは平面上の点だけ。把持は**接地点**から取る（物体上部は誤写）。
   高さ(Z)は観測不能 → §「高さ」で定数化。積み重ね・多平面・非平面は不可。
2. **ボード範囲外は精度急落**: 歪みは領域内の内挿として吸収。**対象はボード内**が必須
   （= 確定制約と一致）。ボード外の素テーブルは取れない。
3. **カメラ/ボードを動かしたら再校正**（単発なので安い。常設運用なら実害小）。
4. **校正フレームのコーナー被覆に依存**: アーム退避で全面が見える1枚を使う。
5. **スケール**: `square_length_mm` 実測が原則（§物理ボード）。`--scale` が安全網。
6. **board→base タッチ校正は不変**（ロボット側の制約は同じ）。

→ これらが捨てるもの（ボード外ピック/高さ/多平面/内部パラ再利用）は、**今回の確定制約
では全部すでに対象外**。実用損失ゼロで単発校正・~0.5mm・API/MCP化容易の利点を得る。

---

## item 1 — タッチ点の点数・配置（board→base）

剛体フィットの誤差は概ね `err ∝ noise / (√N · spread)`。共線は脆い。
`touch_calibrate.py --simulate`（ノイズ1mm, 400試行, 10×7コーナー）実測：

| N | pred RMS(mm) | pred p95(mm) |
|---|---|---|
| 3 | 1.19 | 2.29 |
| 4 | 0.97 | 1.72 |
| **6** | **0.81** | 1.48 |
| 9 | 0.66 | 1.20 |

→ 4隅→中央→辺中点で全域へ広げ **N=6** が頃合い（`recommend_touch_corner_ids` が自動選択）。

---

## item 2 — カメラ配置ガイド

感度は `(R²/h)`（R=斜距離, h=ボード上高さ）。`placement_calculator.py` で事前確認。
**接地点検出ならパララックス0で h=400–1000mm どこでも合格（~3.6–4.0mm, ロボット律速）**。
それでも**真上寄り・近く・中央寄せ・フル解像**が望ましい。今回のカメラは調整後かなり
真上寄りになり、ボードが画面中央に大きく写る良好な状態。

---

## item 3 — 物体検出

**空ボード参照との差分**（`--save-ref` → absdiff → Otsu → モルフォロジ → 最大輪郭）。静的＆
既知背景で堅牢・ML不要。把持画素は**重心**（真上寄り構成では接地点とほぼ一致; 平面外
パララックス小）。yaw は minAreaRect 長軸をホモグラフィでボードへ写し base 角度に変換
（`solve_topdown_ik` は現状 yaw 無視なので情報提供）。

---

## 高さ（Z）— 制約として固定

単眼+平面は XY を解くが物体高は観測不能。**把持 Z = ボード平面（board→base で既知）+
クラス既定オフセット**（`config.GRASP.grasp_z_offset_mm`）で固定。大物体前提なので一定
オフセット上面把持で捉える。

---

## 検証状況（2026-06-07, 実機・実画像）

| 項目 | 結果 |
|---|---|
| 純幾何（Kabsch/Umeyama scale/homography/chain） | ✅ 自己テスト合格 |
| 配置計算 / タッチ点シム | ✅ 合格 |
| Pi 環境 | ✅ cv2 4.12, `/dev/ttyACM0` 接続, `feat-vision` checkout |
| SORACAM フレーム（MCP, 1920×1080） | ✅ 取得・デコード可 |
| ボード特定（10×7, 6x6_250） | ✅ 候補総当りで一意 |
| homography 校正（実フレーム） | ✅ LOO mean 0.53mm |
| locate フル経路（実フレーム＋合成物体） | ✅ 中心(126,84)mm→復元(125.9,84.2)mm |
| **board→base タッチ校正（実機）** | ⏳ 未実施（要対話操作; §B） |
| **square_length_mm 実測** | ⏳ 未測定（スケール基準） |

---

## ファイル構成（`scripts/vision/`）

| ファイル | 役割 |
|---|---|
| `config.py` | ボード仕様(10×7,6x6_250)・パス・フレーム源・把持高さ |
| `geometry.py` | apply_homography / umeyama(scale) / kabsch / 変換（純numpy・自己テスト） |
| `placement_calculator.py` | item2 配置ガイド（純math・自己テスト） |
| `charuco_board.py` | ボード定義 + 印刷PNG生成 + 検出器 |
| `frame_source.py` | RTSP/DASH + MCP-still |
| `homography_calibrate.py` | **A: 単発 画素↔ボード ホモグラフィ + LOO残差** |
| `touch_calibrate.py` | **B: board→base タッチ（Umeyama, --scale）+ --simulate(item1)** |
| `locate.py` | **C: 物体検出→ベース系座標（locate契約）** |

実行手順は `scripts/vision/README.md`。

---

## 未検証・open items

- `touch_calibrate.py --collect` のフリードライブ手順（`release_all_servos`→手引き→Enter→
  `corrected_fk`）は実機未検証。手保持のたわみ次第で N を増やす。
- `square_length_mm` 実測（または `--scale` 運用）。
- 物体検出は実物体での確認が未（合成物体までは確認済）。照明・影の閾値調整余地。
- API/MCP 化: 上記関数群を REST（`mycobot_api_server.py`）と MCP（`mcp-server/`）から呼べる
  よう薄いラッパを追加する段が次。校正成果物は `scripts/vision/calib/` に永続化される。
