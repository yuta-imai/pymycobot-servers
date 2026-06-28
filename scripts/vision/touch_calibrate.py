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
import os
from typing import List

import numpy as np

import config
from geometry import (kabsch, umeyama, rigid_fit_residuals, transform_points,
                      make_transform, apply_homography, apply_plane, fit_plane)


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

    # Close the gripper so the contact point is the defined, repeatable closed
    # fingertip. KEEP THE GRIPPER VERTICAL (pointing straight down) at every
    # touch: a constant approach orientation makes the (frame6 -> fingertip)
    # offset a constant vector that the board->base fit absorbs. Varying the
    # orientation scatters the points (that was the 21mm-RMS failure mode).
    try:
        ctrl.close_gripper(speed=50, gripper_type=3)
        import time as _t; _t.sleep(1.0)
        print("[touch] gripper closed (parallel, type=3)")
    except Exception as exc:
        print(f"[touch] WARN: close_gripper failed ({exc}); close it by hand")

    src_board, dst_base, used_ids = [], [], []
    try:
        for k, cid in enumerate(ids):
            bx, by, _ = corners[cid]
            print(f"\n[{k+1}/{len(ids)}] corner id={cid}  board=({bx:.1f},{by:.1f}) mm")
            print("  releasing servos — hold the gripper VERTICAL (straight down) "
                  "and put the closed fingertip on that corner; hold steady.")
            print("  press Enter to sample  /  s=skip (unreachable)  /  q=abort")
            ctrl.mc.release_all_servos()
            ans = input("  > ").strip().lower()
            if ans == "q":
                raise KeyboardInterrupt
            if ans == "s":
                print("    skipped")
                continue
            angles = ctrl.get_all_joint_angles()
            tip = ctrl.corrected_fk(angles)[:3]
            downness = -ctrl.corrected_approach_axis(angles)[2]
            flag = "" if downness >= 0.9 else "  <-- NOT vertical! re-do more upright"
            print(f"    angles={['%.1f' % a for a in angles]}")
            print(f"    tip(base)={[round(v,1) for v in tip]} mm  "
                  f"downness={downness:+.2f}{flag}")
            src_board.append([bx, by, 0.0])
            dst_base.append(tip)
            used_ids.append(cid)
    finally:
        try:
            ctrl.mc.power_on()
        except Exception:
            pass
        ctrl.disconnect() if hasattr(ctrl, "disconnect") else None

    if len(src_board) < 4:
        raise SystemExit(f"need >= 4 points for a homography, got {len(src_board)}")
    import cv2
    src = np.array(src_board, float)
    dst = np.array(dst_base, float)
    src_xy, base_xy, base_z = src[:, :2], dst[:, :2], dst[:, 2]
    # board(X,Y) -> base(x,y) as a HOMOGRAPHY (absorbs the arm's projective
    # corrected_fk distortion; a rigid fit fails on this unit), RANSAC rejects
    # mis-touches; base z as a tilted plane. See geometry.base_grasp_point.
    H, mask = cv2.findHomography(src_xy, base_xy, cv2.RANSAC,
                                 ransacReprojThreshold=args.ransac_mm)
    if H is None:
        raise SystemExit("homography fit failed (degenerate/too few points?)")
    inl = mask.ravel().astype(bool)
    if inl.sum() < 4:
        print("  NOTE: RANSAC kept <4; falling back to all points")
        inl = np.ones(len(src), bool)
    z_plane = fit_plane(src_xy[inl], base_z[inl])
    xy_res = np.linalg.norm(apply_homography(H, src_xy) - base_xy, axis=1)
    z_res = np.abs(apply_plane(z_plane, src_xy) - base_z)
    xy_rms = float(np.sqrt(np.mean(xy_res[inl] ** 2)))
    z_rms = float(np.sqrt(np.mean(z_res[inl] ** 2)))
    print(f"\n[fit] board->base = homography(xy)+plane(z)  "
          f"{int(inl.sum())}/{len(src)} inliers")
    print(f"      xy residual: rms={xy_rms:.2f} max={xy_res[inl].max():.2f} mm")
    print(f"      z  residual: rms={z_rms:.2f} max={z_res[inl].max():.2f} mm")
    for cid, e, ok in zip(used_ids, xy_res, inl):
        print(f"   corner {cid}: xy {e:.2f} mm" + ("" if ok else "  <-- OUTLIER (rejected)"))
    if xy_rms > 4.0:
        print("WARN: xy rms > 4 mm — add more spread points or re-touch outliers.")

    config.ensure_artifact_dir()
    out = config.artifact_path(config.HANDEYE_JSON)
    with open(out, "w") as f:
        json.dump({"model": "homography+plane",
                   "H_board2base": H.tolist(), "z_plane": z_plane.tolist(),
                   "xy_rms_mm": xy_rms, "xy_max_mm": float(xy_res[inl].max()),
                   "z_rms_mm": z_rms, "n_points": len(src),
                   "n_inliers": int(inl.sum()), "inliers": inl.tolist(),
                   "corner_ids": used_ids,
                   "src_board_mm": src.tolist(),
                   "dst_base_mm": dst.tolist()}, f, indent=2)
    print(f"wrote {out}")


def load_board_to_base(path: str = None) -> dict:
    """Return {H_board2base, z_plane (3,)} for board(X,Y)->base(x,y,z).

    H_board2base is whatever geometry.board_to_base_fn accepts: a 3x3 homography
    (model "homography+plane") OR the full model dict (model "rbf-*", which is
    NON-projective and fits much better — see calib/board_to_base_rbf.json).

    If no explicit path is given, prefer the RBF calibration when present
    (config.HANDEYE_RBF_JSON), else fall back to the homography one.
    """
    if path is None:
        rbf = config.artifact_path(getattr(config, "HANDEYE_RBF_JSON",
                                           "board_to_base_rbf.json"))
        path = rbf if os.path.exists(rbf) else config.artifact_path(config.HANDEYE_JSON)
    with open(path) as f:
        d = json.load(f)
    if str(d.get("model", "homography+plane")).startswith("rbf"):
        # RBF map carries no fitted z-plane; use a flat plane at the hover/command
        # z it was sampled at. NOTE: that is a HOVER height, not the board surface
        # — grasp z still needs a touch calibration (touch_calibrate --collect).
        zc = float(d.get("verified", {}).get("hover_z_cmd", d.get("z_cmd", 90.0)))
        return {"H_board2base": d, "z_plane": np.array([0.0, 0.0, zc], float)}
    return {"H_board2base": np.array(d["H_board2base"], float),
            "z_plane": np.array(d["z_plane"], float)}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true",
                      help="design study: point-count/spread vs fit error")
    mode.add_argument("--collect", action="store_true",
                      help="live touch calibration on the robot host")
    ap.add_argument("--n", type=int, default=9, help="number of touch points "
                    "(>=8 recommended for a robust homography + outlier rejection)")
    ap.add_argument("--corner-ids", default=None,
                    help="explicit comma-separated ChArUco corner ids to touch, "
                         "in order (matches a numbered reference image); "
                         "overrides --n/auto-pick")
    ap.add_argument("--ransac-mm", type=float, default=8.0,
                    help="RANSAC reprojection threshold (mm) for outlier rejection")
    ap.add_argument("--scale", action="store_true", help="(deprecated, ignored)")
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
