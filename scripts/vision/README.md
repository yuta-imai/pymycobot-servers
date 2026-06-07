# `scripts/vision/` — Eye-to-Hand SORACAM ピック準備物

単眼 SORACAM + A3 ChArUco でトップダウン・ピックの座標を出すための一式。
設計と誤差解析は [`docs/eye-to-hand-soracam.md`](../../docs/eye-to-hand-soracam.md)。

- **純幾何**（`geometry.py` / `placement_calculator.py` / `touch_calibrate.py --simulate`）
  は numpy/scipy のみ。どこでも動く。
- **cv2 依存**（`charuco_board` / `frame_source` / `calibrate_*` / `locate`）と
  **ロボット依存**（`touch_calibrate --collect`）は **Pi / ロボットホスト**で。
  `pip install -r requirements.txt`（`opencv-python` を含む）。

校正成果物は `scripts/vision/calib/` に出力される（intrinsics / extrinsics / board_to_base）。

## 0. 事前検討（ハード前に数字で確認）

```bash
python placement_calculator.py --fov 120 --width 1920 --tol-mm 8 --work-radius 210
python touch_calibrate.py --simulate                 # 点数 vs 誤差
```

## 1. 物理セットアップ

```bash
python charuco_board.py --out charuco_A3.png --dpi 300
```
→ **100% スケールで A3 印刷** → 1 マスをノギスで実測 → `config.py` の
`BOARD.square_length_mm` を実測値に更新。アームとボードを固定（カメラは真上寄り推奨）。

## 2. キャリブレーション（1 回）

環境変数（rtsp バックエンド時）:
```bash
export SORACAM_STREAM_URL="rtsp://..."   # or DASH URL from MCP get_live_view_unlimited
```

```bash
# 2a 内部+歪み: ボードを色々動かして連写 → 校正
python frame_source.py --backend rtsp --burst 25 --interval 0.8 --out-dir captures/
python calibrate_intrinsics.py --images captures/ --model rational   # RMS>1px なら --model fisheye

# 2b camera->board: ボードを作業位置に平置きして単発 PnP
python calibrate_extrinsics.py --grab            # or --image board_flat.png

# 2c board->base: 既知コーナーを 6 点タッチ（ロボットホストで）
python touch_calibrate.py --collect --n 6 --port /dev/ttyACM0
```

## 3. ピック（毎回）

```bash
# 一度だけ: 物体を置く前の空ボードを参照として保存
python locate.py --save-ref --grab

# 物体を置いて座標を取得（locate 契約 JSON）
python locate.py --grab --json
# -> {"ok": true, "base_xyz_mm": [x,y,z], "yaw_deg": .., ...}
```
`base_xyz_mm` をそのまま `solve_topdown_ik(x,y,z)` → `send_angles` に渡す
（壊れている firmware `send_coords` は使わない）。ドリフトが気になれば
`locate.py --grab --refresh-pnp` で毎回 camera→board を取り直す。

## メモ

- `mcp` バックエンド（一発グランス用）: soracam MCP `get_live_still_image` の結果 JSON を
  ファイル保存し `--backend mcp --mcp-json result.json`。連写には向かない（~15s/枚）。
- 高さは観測せず `config.GRASP.grasp_z_offset_mm` 固定（設計上の制約）。
- 1 プロセスのみロボットシリアルを保持できる。`locate` 実行とロボット制御の同時保持に注意。
- OpenCV aruco API は 4.7 前後で差。新旧両対応で書いてあるが実機バージョンで要確認。
