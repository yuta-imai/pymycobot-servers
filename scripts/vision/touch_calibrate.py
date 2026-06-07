#!/usr/bin/env python3
"""Step 2c: board->base rigid transform by touching known ChArUco corners (item 1).

RUNS ON THE ROBOT HOST. For each chosen board corner (known board-frame mm),
hand-guide the gripper TIP onto the printed corner and sample: we read joint
angles and compute the tip position in base frame via the accelerometer-
calibrated model (controller.corrected_fk -- the trusted FK; firmware get_coords
is NOT used). Kabsch then fits board->base.

Why this is robust here: solve_topdown_ik targets the SAME corrected model used
to sample, so any rigid FK bias is folded into board->base and CANCELS at pick
time. Only non-rigid FK error survives (second order).

    # design study (no robot, no cv2): how many points / how spread?
    python touch_calibrate.py --simulate

    # live calibration (robot host, project venv); freedrive between samples:
    python touch_calibrate.py --collect --n 6 --port /dev/ttyACM0

Writes config.calib/board_to_base.json = {T_base_base (4x4), rms_mm, points...}.
"""

from __future__ import annotations

import argparse
import json
from typing import List

import numpy as np

import config
from geometry import (kabsch, umeyama, rigid_fit_residuals, transform_points,
                      make_transform)


# --------------------------------------------------------------------------- #
# Item 1: which corners to touch (spread for good conditioning)
# --------------------------------------------------------------------------- #
def recommend_touch_corner_ids(corners_mm: np.ndarray, n: int) -> List[int]:
    """Pick n well-spread, non-collinear corner ids from the board grid.

    Targets the 4 extreme corners first (fix rotation about every axis), then the
    center, then edge midpoints. Spread over the full A3 extent minimizes the
    rotational error of the rigid fit (error ~ noise / (sqrt(N) * spread)).
    """
    xy = corners_mm[:, :2]
    lo, hi = xy.min(0), xy.max(0)
    mid = (lo + hi) / 2.0
    targets = [
        (lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]),  # corners
        (mid[0], mid[1]),                                                # center
        (mid[0], lo[1]), (hi[0], mid[1]), (mid[0], hi[1]), (lo[0], mid[1]),  # edges
    ]
    chosen: List[int] = []
    for tx, ty in targets[:max(4, n)]:
        d = np.hypot(xy[:, 0] - tx, xy[:, 1] - ty)
        for idx in np.argsort(d):
            if int(idx) not in chosen:
                chosen.append(int(idx))
                break
        if len(chosen) >= n:
            break
    return chosen[:n]


# --------------------------------------------------------------------------- #
# Item 1: Monte-Carlo design study (pure numpy; runs anywhere)
# --------------------------------------------------------------------------- #
def simulate(noise_mm: float = 1.0, trials: int = 400, seed: int = 0):
    """How board->base fit error scales with point count and spread.

    Synthesizes a true rigid transform, samples touch points with isotropic
    Gaussian noise, fits with Kabsch, and reports the RMS *prediction* error at
    board test points (what actually hits the pick). Answers '4 vs 6 vs 9?'.
    """
    from charuco_board import build_board, chessboard_corners_mm  # cv2 optional...
    try:
        _, board = build_board()
        corners = chessboard_corners_mm(board)
    except Exception:
        # synthesize an A3-like 7x10 corner grid if cv2/board is unavailable
        xs = np.linspace(0, config.BOARD.width_mm, config.BOARD.squares_x - 1)
        ys = np.linspace(0, config.BOARD.height_mm, config.BOARD.squares_y - 1)
        gx, gy = np.meshgrid(xs, ys)
        corners = np.column_stack([gx.ravel(), gy.ravel(),
                                   np.zeros(gx.size)])

    rng = np.random.default_rng(seed)
    # a plausible board->base: rotate ~40deg in plane, offset into the workspace
    th = np.radians(40)
    R = np.array([[np.cos(th), -np.sin(th), 0],
                  [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    T_true = make_transform(R, np.array([180.0, -40.0, -30.0]))
    test_pts = corners.copy()                          # evaluate everywhere

    print(f"Monte-Carlo board->base fit  (touch noise = {noise_mm} mm RMS, "
          f"{trials} trials)")
    print(f"{'N':>3}{'spread':>8}  {'pred_rms_mm':>12}{'pred_p95_mm':>12}")
    print("-" * 38)
    for n in (3, 4, 6, 9):
        ids = recommend_touch_corner_ids(corners, n)
        src = corners[ids]
        spread = float(np.linalg.norm(src[:, :2] - src[:, :2].mean(0),
                                      axis=1).mean())
        errs = []
        for _ in range(trials):
            dst = transform_points(T_true, src) + rng.normal(0, noise_mm, src.shape)
            T_fit = kabsch(src, dst)
            pred = transform_points(T_fit, test_pts)
            truth = transform_points(T_true, test_pts)
            errs.append(np.linalg.norm(pred - truth, axis=1))
        errs = np.concatenate(errs)
        print(f"{n:>3}{spread:8.0f}  {errs.mean():12.2f}{np.percentile(errs,95):12.2f}")
    print("\nReading it: error falls ~1/sqrt(N) and with spread. N=6 over the full")
    print("A3 extent is the sweet spot; N=3 is collinear-fragile, N=9 barely helps.")


# --------------------------------------------------------------------------- #
# Live collection (robot host)
# --------------------------------------------------------------------------- #
def collect(args):
    import sys
    import os
    sys.path.insert(0, config.ROOT)
    from mycobot_joint_controller import MyCobotJointController
    from charuco_board import build_board, chessboard_corners_mm

    _, board = build_board()
    corners = chessboard_corners_mm(board)
    if args.corner_ids:
        ids = [int(x) for x in args.corner_ids.split(",")]
    else:
        ids = recommend_touch_corner_ids(corners, args.n)
    print(f"[touch] will touch {len(ids)} corners in this order (ids={ids})")
    print("[touch] follow the numbered annotated reference image for each point")

    ctrl = MyCobotJointController(port=args.port, baudrate=args.baudrate)
    if ctrl._cm_dh is None:
        raise SystemExit(f"corrected model unavailable: {ctrl._cm_error}")

    src_board, dst_base = [], []
    try:
        for k, cid in enumerate(ids):
            bx, by, _ = corners[cid]
            print(f"\n[{k+1}/{len(ids)}] corner id={cid}  board=({bx:.1f},{by:.1f}) mm")
            print("  releasing servos — hand-guide the TIP onto that printed "
                  "corner, hold steady, then press Enter (q=abort)...")
            ctrl.mc.release_all_servos()
            if input("  > ").strip().lower() == "q":
                raise KeyboardInterrupt
            angles = ctrl.get_all_joint_angles()
            tip = ctrl.corrected_fk(angles)[:3]
            print(f"    angles={['%.1f' % a for a in angles]}")
            print(f"    tip(base)={[round(v,1) for v in tip]} mm")
            src_board.append([bx, by, 0.0])
            dst_base.append(tip)
    finally:
        try:
            ctrl.mc.power_on()
        except Exception:
            pass
        ctrl.disconnect() if hasattr(ctrl, "disconnect") else None

    src = np.array(src_board, float)
    dst = np.array(dst_base, float)
    T, scale = umeyama(src, dst, with_scale=args.scale)
    rms, mx, per = rigid_fit_residuals(T, src, dst)
    print(f"\n[fit] board->base RMS={rms:.2f} mm  max={mx:.2f} mm  "
          f"scale={scale:.4f}{'  (estimated)' if args.scale else '  (fixed=1)'}")
    for cid, e in zip(ids, per):
        print(f"   corner {cid}: residual {e:.2f} mm")
    if args.scale and abs(scale - 1.0) > 0.03:
        print(f"NOTE: scale {scale:.3f} differs from 1 by >3% — your "
              "square_length_mm is likely off; measure and set config.BOARD.")
    if rms > 3.0:
        print("WARN: RMS > 3 mm — re-touch the worst corners or add points.")

    config.ensure_artifact_dir()
    out = config.artifact_path(config.HANDEYE_JSON)
    with open(out, "w") as f:
        json.dump({"T_base_board": T.tolist(), "scale": scale,
                   "scale_estimated": bool(args.scale),
                   "rms_mm": rms, "max_mm": mx, "corner_ids": ids,
                   "src_board_mm": src.tolist(),
                   "dst_base_mm": dst.tolist()}, f, indent=2)
    print(f"wrote {out}")


def load_board_to_base(path: str = None) -> np.ndarray:
    path = path or config.artifact_path(config.HANDEYE_JSON)
    with open(path) as f:
        return np.array(json.load(f)["T_base_board"], float)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true",
                      help="design study: point-count/spread vs fit error")
    mode.add_argument("--collect", action="store_true",
                      help="live touch calibration on the robot host")
    ap.add_argument("--n", type=int, default=6, help="number of touch points")
    ap.add_argument("--corner-ids", default=None,
                    help="explicit comma-separated ChArUco corner ids to touch, "
                         "in order (matches a numbered reference image); "
                         "overrides --n/auto-pick")
    ap.add_argument("--scale", action="store_true",
                    help="estimate a similarity scale (safety net for an "
                         "unmeasured square_length_mm); default rigid (scale=1)")
    ap.add_argument("--noise-mm", type=float, default=1.0, help="(simulate) touch noise")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baudrate", type=int, default=115200)
    args = ap.parse_args()

    if args.simulate:
        simulate(noise_mm=args.noise_mm)
    else:
        collect(args)


if __name__ == "__main__":
    main()
