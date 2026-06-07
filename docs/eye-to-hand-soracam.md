# Eye-to-Hand 視覚ピック — 単眼 SORACAM + A3 ChArUco

D435（深度カメラ）を使わず、**固定した単眼 SORACAM 1台 + 既知平面（A3 ChArUco）**で
トップダウン・ピックを実現するための設計と準備物。対象は「ある程度大きい物体」で、
セットアップ後の環境は静的、という前提（本プロジェクトの確定条件）。

実装は `scripts/vision/`。純幾何（`geometry.py` / `placement_calculator.py` /
`touch_calibrate.py --simulate`）は numpy/scipy のみで、開発機で自己テスト済み。
OpenCV / ロボット依存部分は Pi・ロボットホスト上で実行する。

---

## なぜ深度が要らないか

対象が**既知平面（ボード面）上**にあるなら、画素レイをその平面と交差させるだけで 3D が
出る。深度（D435 やステレオ）は不要。さらに配置誤差解析（§2）から、**適切に置けば視覚
誤差はこの 280 アーム自身の実行誤差（~3.5mm）より小さく**、視覚は律速にならない。
→ D435 を入れてもアームが精度を使い切れないため、単眼+平面で必要十分。

詳細な比較検討（ソラカメ読み取り可否、ステレオ vs 平面、D435 との優劣）は本ドキュメント
の前段検討に基づく。要点：
- soracam MCP の `get_live_view_unlimited` は **DASH URL を返すだけ**で、私（エージェント）
  からはフレームを直接は読めない。`get_live_still_image` は base64 JPEG（デコードすれば可読
  だが ~15s/枚・大きい）。→ **フレーム取得は Pi 側 OpenCV が RTSP/DASH を直接食う**設計。
- クラウドカメラは同期もハード genlock も無く低遅延でもない → **「一度見て→計画→
  オープンループ実行」**専用（閉ループ・ビジュアルサーボは非対象）。

---

## 座標チェーンと「バイアス相殺」性質（設計の核）

```
画素 ──undistort(K,dist)──▶ 正規化座標 ──▶ カメラ系レイ
                                              │  ∩ ボード平面 (PnP: T_cam_board)
                                              ▼
                                        ボード系 (x,y,0)
                                              │  × board→base (タッチ校正: T_base_board)
                                              ▼
                                        ベース系 (X,Y,Z) ──▶ solve_topdown_ik ──▶ send_angles
```

- **camera→board** = A3 ChArUco の単発 PnP（多マーカー・過剰決定で堅牢）。
- **board→base** = ChArUco 既知コーナーを**グリッパ先端でタッチ**し、`corrected_fk`
  （accel 校正済みの信頼できる FK）で先端ベース座標を読み、Kabsch で剛体フィット。
- **核心**: `solve_topdown_ik` の目標一致判定も同じ `corrected_fk`/`_cm_frames` を使う。
  つまり**校正も実行も同一運動学モデル**なので、FK の**剛体的バイアスは board→base に
  畳み込まれてピック時に相殺**する。残るのは非剛体誤差（二次）だけ。firmware FK/IK が
  壊れているこの個体に最も適した構成。

---

## ワークフロー（3 ステップ）

### ① 物理セットアップ
- アーム固定。SORACAM 固定（**できるだけ真上寄り・近く**＝§2）。
- A3 ChArUco を作業位置に平置きし、**以後そのまま作業面として残す**。
- ボードは `charuco_board.py` で生成し **100% スケールで A3 印刷**、1 マスをノギスで実測して
  `config.BOARD.square_length_mm` に反映（印刷スケール誤差はほぼ 1:1 でmm誤差になる）。

### ② キャリブレーション（セットアップ後 1 回）
- **2a 内部+歪み** `calibrate_intrinsics.py`：カメラ固定のままボードを色々な姿勢で動かして
  ~20–30 枚撮る（`frame_source.py --burst`）。広角 ATOM は既定の **rational（k1..k6）**で吸収、
  残差 RMS が高ければ `--model fisheye`。
- **2b camera→board** `calibrate_extrinsics.py`：ボードを作業位置に平置きして単発 PnP。
- **2c board→base** `touch_calibrate.py --collect`：既知コーナーを**6点**タッチ（§1）。

### ③ ピック（毎回）
- `locate.py`：フレーム取得 →（任意で PnP リフレッシュ）→ 空ボード参照との差分で物体検出
  → **接地点画素** → レイ → ボード平面交差 → ベース系 (X,Y) → `base_xyz_mm` 返却。
- 返却は `docs/integration-plan.md` の `locate` 契約（ベース系・mm/deg）に整合。そのまま
  `solve_topdown_ik` / `pick_at` に渡せる。

---

## item 1 — タッチ点の点数・配置設計

board→base は剛体フィット。誤差伝播は概ね **err ∝ noise / (√N · spread)**：点数を増やすほど
1/√N で減り、ボード全域に広げるほど回転成分が締まる。**共線配置は回転が決まらず脆い**。

`touch_calibrate.py --simulate`（タッチノイズ 1mm RMS, 400 試行, A3 7×10 コーナー）実測：

| N | spread(mm) | pred RMS(mm) | pred p95(mm) |
|---|---|---|---|
| 3 | 219 | 1.19 | 2.29 |
| 4 | 238 | 0.97 | 1.72 |
| **6** | 188 | **0.81** | 1.48 |
| 9 | 182 | 0.66 | 1.20 |

**結論**: 4 隅 → 中央 → 辺中点の順に**全域へ広げて N=6** が頃合い（N=3 は共線的に脆い、
N=9 は伸びが鈍い）。`recommend_touch_corner_ids()` がこの配置を自動選択。タッチノイズ
1mm 前提なら board→base 残差は **~0.8mm**＝バジェット上は小項。

---

## item 2 — カメラ配置ガイド

レイ⇄平面の感度は **(R²/h)**（R=斜距離, h=ボード上高さ）で、斜めになるほど発散。地面誤差は
- 検出/IFOV項: `detect_px · IFOV · (R²/h)`、IFOV=FOV/width
- 校正/レイ項: `calib_mrad · (R²/h)`
- **パララックス項**（重心を使うと）: `(物体高/2)·tan(入射角)` ← 斜めで激増

`placement_calculator.py`（FOV120°, 1920px, detect1px, calib1.5mrad, robot3.5mm,
work_radius210mm, 最遠点で評価）実測：

**重心を使う場合**（パララックスが支配）→ h を上げないと収まらない：
| h(mm) | 入射 | mm/px | parax | tot_rss | 判定(≤8mm) |
|---|---|---|---|---|---|
| 400 | 27.7° | 0.56 | 15.75 | 16.16 | NG |
| 800 | 14.7° | 0.93 | 7.88 | 8.76 | NG |
| 1000 | 11.9° | 1.14 | 6.30 | 7.46 | OK |

**接地点を使う場合**（パララックス=0、既定）→ **どの高さでも余裕**で OK：
| h(mm) | 入射 | vis_rss | tot_rss | 判定 |
|---|---|---|---|---|
| 400 | 27.7° | 0.95 | 3.63 | OK |
| 800 | 14.7° | 1.59 | 3.84 | OK |
| 1000 | 11.9° | 1.94 | 4.00 | OK |

**結論**:
1. **接地点検出が最重要**（`locate.py` 既定）。これでパララックスが消え、配置は寛容に。
2. それでも**真上寄り・近く**が望ましい（全項を縮め、平面高誤差 `δh·tanα` も抑える）。
3. **フル解像取得・ピック帯を画像中央寄せ**（歪み残差最小域）。

自分の構成で確認するには：
```
python placement_calculator.py --fov 120 --width 1920 --tol-mm 8 --work-radius 210
```

---

## item 3 — 物体検出

静的＆既知平面を最大活用：**空ボード参照フレームを1枚撮っておき、ピック時に差分**を取る
（`locate.py --save-ref` → 以後 absdiff → Otsu → モルフォロジ → 最大輪郭）。ML 不要・堅牢。
ChArUco は部分遮蔽に強いので、物体が一部マーカーを隠しても PnP リフレッシュ可能。

- **接地点画素**＝輪郭点のうち**カメラ直下（nadir）方向に最も近い点**を採用。真上寄り配置
  では重心とほぼ一致、斜め配置では基部エッジを拾い、重心パララックスを消す（`--centroid`
  で重心法に切替可）。
- **yaw** は minAreaRect の長軸をボード平面へ逆投影 → ベース系角度に変換して返す
  （`solve_topdown_ik` は現状 yaw 無視なので情報提供）。

---

## 高さ（Z）の扱い — 制約として固定

単眼+平面は **XY は解くが物体の高さ（Z 寸法）は観測できない**。本構成では設計上の制約と
して、**把持 Z = ボード平面（タッチ校正で既知）+ クラス既定オフセット**で固定する
（`config.GRASP.grasp_z_offset_mm`）。大物体前提なので一定オフセット上面把持で捉えられる。

---

## 誤差バジェット（まとめ・推奨配置 + 接地点検出）

| 成分 | 値 |
|---|---|
| camera→board PnP | サブmm〜~1mm（A3・内挿） |
| 歪み残差 | ~0.4mm |
| board→base タッチ（N=6） | ~0.8mm（1回・固定→再現性に転化） |
| 検出/IFOV | ~1mm |
| **ロボット実行** | **~3.5mm（支配項）** |
| **合計 RSS** | **~3.6–4.0mm（ロボット律速）** |

大物体＋緩い把持トレランスなら余裕。

---

## ファイル構成（`scripts/vision/`）

| ファイル | 役割 | 開発機テスト |
|---|---|---|
| `config.py` | ボード仕様・パス・フレーム源・把持高さ（単一の真実） | import OK |
| `geometry.py` | Kabsch / レイ⇄平面 / 変換（純numpy） | **自己テスト合格** |
| `placement_calculator.py` | item2 配置ガイド（純math） | **自己テスト合格** |
| `charuco_board.py` | A3 ボード定義 + 印刷PNG生成 + 検出器 | 要 cv2 |
| `frame_source.py` | RTSP/DASH + MCP-still バックエンド | 要 cv2 |
| `calibrate_intrinsics.py` | 2a 内部+歪み（rational/fisheye） | 要 cv2 |
| `calibrate_extrinsics.py` | 2b camera→board PnP | 要 cv2 |
| `touch_calibrate.py` | 2c board→base タッチ + `--simulate`(item1) | simulate合格 / collect要robot |
| `locate.py` | 3 物体検出→ベース系座標（locate契約） | 要 cv2 |

実行手順は `scripts/vision/README.md`。

---

## 未検証・open items

- cv2 依存部分（`charuco_board`/`calibrate_*`/`locate`）は開発機に cv2 が無く**未実行**。
  Pi・ロボットホスト（`pip install opencv-python`）で動かして要確認。OpenCV の aruco API は
  4.7 前後で差があり、新旧両対応で書いてあるが実機バージョンで要確認。
- `touch_calibrate.py --collect` の**フリードライブ手順**（`release_all_servos` で手引き
  → Enter で `corrected_fk` サンプル）は実機未検証。手保持のたわみ次第で N を増やす。
- rational モデルで広角歪みが取り切れるかは実画像の RMS 次第（>1px なら fisheye）。
- 接地点 nadir 法の妥当性は実機の取り付け角で要確認（真上寄りなら問題小）。
- シリアルは 1 プロセス占有（既知）。カメラ取得とロボット制御のプロセス分離に注意。
