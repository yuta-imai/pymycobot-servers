# `scripts/vision/` — Eye-to-Hand SORACAM ピック（ホモグラフィ方式）

単眼 SORACAM + A4 ChArUco（10×7, DICT_6X6_250）でトップダウン・ピックの座標を出す一式。
設計と検証は [`docs/eye-to-hand-soracam.md`](../../docs/eye-to-hand-soracam.md)。

- **純幾何**（`geometry.py` / `placement_calculator.py` / `touch_calibrate.py --simulate`）は
  numpy/scipy のみ。どこでも動く。
- **cv2 依存**（`charuco_board` / `frame_source` / `homography_calibrate` / `locate`）と
  **ロボット依存**（`touch_calibrate --collect`）は **Pi / ロボットホスト**で
  （`pip install -r requirements.txt`）。

校正成果物は `scripts/vision/calib/` に出力（`homography.json` / `board_to_base.json`）。

## 0. 事前検討（ハード前に数字で確認）

```bash
python placement_calculator.py --fov 120 --width 1920 --tol-mm 8 --work-radius 210
python touch_calibrate.py --simulate          # タッチ点数 vs 誤差
```

## 1. 物理セットアップ
アームとボードを固定（カメラは真上寄り推奨）。A4 ChArUco を作業位置に常設。
**1マスを実測** → `config.py` の `BOARD.square_length_mm` を実測値に更新。

ボードを印刷する場合は **真スケール PDF** を生成（A4・mm 埋め込み・50mm検証バー付き）：
```bash
# 既定(config.BOARD)で出力
python board_to_pdf.py --out charuco_A4.pdf
# 検出に強い版（粗い辞書＋大きいマーカー; 遠め/低解像でも全面検出しやすい）
python board_to_pdf.py --dict DICT_4X4_50 --squares 10x7 --square-mm 28 --marker-mm 22 \
    --out charuco_4x4_A4.pdf
```
**100% スケールで印刷**（fit to page にしない）→ 50mm バーを定規で確認 → `config.BOARD`
を印刷した board に合わせる（dict / squares / square_length_mm / marker_length_mm）。
注: 既存の 6×6_250 ボードは ~46px のマーカーが小さく、この距離だと約半分しかデコード
できず**ボードの約4割しか検出できない**。全面で使うなら 4×4 版の再印刷を推奨。

## 2. キャリブレーション（1 回）

```bash
export SORACAM_STREAM_URL="rtsp://..."     # rtsp/DASH。MCP still を使うなら frame_source --backend mcp

# A. ホモグラフィ（アーム退避でボード全面が見える1枚）
python homography_calibrate.py --grab        # or --image board_clear.png
#   -> LOO residual mean/p95/max が出る。mean<2mm を確認。

# B. board->base タッチ（ロボットホスト, 6点）
python touch_calibrate.py --collect --n 6 --port /dev/ttyACM0
#   square 未実測なら --scale を付けてスケール推定（|scale-1|>3% なら square がズレている）
```

## 3. ピック（毎回）

```bash
python locate.py --save-ref --grab           # 一度だけ: 物体を置く前の空ボード
python locate.py --grab --json               # 物体を置いて座標取得（locate 契約 JSON）
# -> {"ok": true, "base_xyz_mm": [x,y,z], "board_xy_mm":[..], "yaw_deg": .., ...}
```
`base_xyz_mm` を `solve_topdown_ik(x,y,z)` → `send_angles` へ（壊れている firmware
`send_coords` は使わない）。ドリフトが気になれば `locate.py --grab --refresh`。

## メモ

- `mcp` バックエンド（一発取得）: soracam MCP `get_live_still_image` の結果 JSON を保存して
  `--backend mcp --mcp-json result.json`（~15s/枚、連続用途には不向き）。
- 高さは観測せず `config.GRASP.grasp_z_offset_mm` 固定（設計制約）。
- ロボットシリアルは 1 プロセス占有。`locate` とロボット制御の同時保持に注意。
- 次段: これら関数を API（`mycobot_api_server.py`）/ MCP（`mcp-server/`）から呼ぶ薄いラッパ。
