#!/usr/bin/env python3
"""Initial-setup calibration routine — re-run this whenever the camera or arm moves.

RUNS ON THE ROBOT HOST. One command drives the whole eye-to-hand calibration:

  [1] HOMOGRAPHY + reference: grab one board frame (arm clear of the board),
      fit the pixel<->board homography, save it AND the frame as the empty-board
      reference for locate. Also auto-picks well-spread touch targets and writes a
      numbered reference image for step 2.
  [2] BOARD->BASE touch: close the gripper, then for each numbered target release
      servos, you hand-guide the VERTICAL closed gripper onto that corner, Enter
      to sample; fits board->base (Umeyama, optional scale).
  [3] SUMMARY: prints both artifacts' quality and what is still TODO.

Frames: --board-image FILE, or the SORACOM still API (set SORACOM_AUTH_KEY_ID /
SORACOM_AUTH_KEY / SORACOM_DEVICE_ID, coverage via SORACOM_COVERAGE=jp|g), or any
configured frame_source backend.

    python setup_calibration.py                      # full routine
    python setup_calibration.py --board-image b.png  # provide the frame
    python setup_calibration.py --only homography
    python setup_calibration.py --only touch --corner-ids 0,6,21,46,53

Artifacts land in scripts/vision/calib/ (homography.json, board_to_base.json,
empty_board_ref.png, touch_targets.png).
"""

from __future__ import annotations

import argparse
import json
import os
from types import SimpleNamespace

import numpy as np

import config


# --------------------------------------------------------------------------- #
def _grab_frame(args):
    import cv2
    if args.board_image:
        img = cv2.imread(args.board_image)
        if img is None:
            raise SystemExit(f"cannot read {args.board_image}")
        return img
    from frame_source import grab
    print(f"[grab] backend={args.backend or config.FRAME_SOURCE.backend} ...")
    return grab(backend=args.backend)


def _select_spread(corners_mm, ids, n):
    """Farthest-point sampling over DETECTED corners for a spread touch set."""
    bd = corners_mm[ids][:, :2]
    start = int(np.argmax(np.linalg.norm(bd - bd.mean(0), axis=1)))
    chosen = [start]
    for _ in range(min(n, len(ids)) - 1):
        d = np.min([np.linalg.norm(bd - bd[c], axis=1) for c in chosen], axis=0)
        chosen.append(int(np.argmax(d)))
    return [int(ids[i]) for i in chosen]


def step_homography(args) -> dict:
    import cv2
    from charuco_board import build_board, make_detector, chessboard_corners_mm
    import homography_calibrate as hc

    print("\n=== [1] HOMOGRAPHY + reference ===")
    print("  Make sure the ARM IS CLEAR of the board (full board visible).")
    if not args.board_image and not args.yes:
        input("  press Enter to grab the board frame...")
    frame = _grab_frame(args)
    print(f"  frame {frame.shape[1]}x{frame.shape[0]}")

    res = hc.fit_from_frame(frame)
    config.ensure_artifact_dir()
    with open(config.artifact_path(config.HOMOGRAPHY_JSON), "w") as f:
        json.dump(res, f, indent=2)
    # the same arm-clear frame is the locate empty-board reference
    ref_path = config.artifact_path("empty_board_ref.png")
    cv2.imwrite(ref_path, frame)
    r = res["loo_residual_mm"]
    print(f"  corners={res['n_corners']}/{config.BOARD.n_corners}  "
          f"LOO mean={r['mean']:.2f} p95={r['p95']:.2f} max={r['max']:.2f} mm")
    print(f"  saved {config.HOMOGRAPHY_JSON} + empty_board_ref.png")
    if r["mean"] > 2.0:
        print("  WARN: residual high — center the board in the frame / improve light.")

    # auto-pick + annotate touch targets from the same frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, board = build_board()
    cc, ci = make_detector(board)(gray)
    ids = ci.flatten()
    corners_mm = chessboard_corners_mm(board)
    sel = _select_spread(corners_mm, ids, args.n)
    px = cc.reshape(-1, 2)
    idpos = {int(v): i for i, v in enumerate(ids)}
    g = 1.6
    lut = np.array([((i / 255.0) ** (1 / g)) * 255 for i in range(256)], np.uint8)
    vis = cv2.LUT(frame, lut)
    for k, cid in enumerate(sel, 1):
        p = tuple(np.round(px[idpos[cid]]).astype(int))
        cv2.circle(vis, p, 20, (0, 0, 255), 4)
        cv2.rectangle(vis, (p[0] + 16, p[1] - 44), (p[0] + 60, p[1] - 6),
                      (255, 255, 255), -1)
        cv2.putText(vis, str(k), (p[0] + 20, p[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 4)
    cv2.imwrite(config.artifact_path("touch_targets.png"), vis)
    print(f"  touch targets (order) ids = {sel}")
    print(f"  numbered reference -> {config.artifact_path('touch_targets.png')}")
    res["touch_ids"] = sel
    return res


def step_touch(args, corner_ids):
    import touch_calibrate as tc
    print("\n=== [2] BOARD->BASE touch ===")
    print(f"  touching {len(corner_ids)} numbered targets (see touch_targets.png).")
    print("  keep the closed gripper VERTICAL (straight down) at every point.")
    ns = SimpleNamespace(corner_ids=",".join(map(str, corner_ids)), n=args.n,
                         port=args.port, baudrate=args.baudrate, scale=args.scale)
    tc.collect(ns)


def step_summary():
    print("\n=== [3] SUMMARY ===")
    hp = config.artifact_path(config.HOMOGRAPHY_JSON)
    bp = config.artifact_path(config.HANDEYE_JSON)
    if os.path.exists(hp):
        d = json.load(open(hp))
        r = d["loo_residual_mm"]
        print(f"  homography : {d['n_corners']} corners, LOO mean {r['mean']:.2f} mm")
    else:
        print("  homography : MISSING")
    if os.path.exists(bp):
        d = json.load(open(bp))
        print(f"  board->base: RMS {d['rms_mm']:.2f} mm, scale {d.get('scale',1):.4f}, "
              f"Z-spread {d.get('z_spread_mm', float('nan')):.1f} mm "
              f"({len(d['corner_ids'])} pts)")
        if d.get("scale_estimated") and abs(d.get("scale", 1) - 1) > 0.03:
            print(f"  -> scale {d['scale']:.3f}: MEASURE a square and set "
                  "config.BOARD.square_length_mm")
        if d["rms_mm"] > 3.0:
            print("  -> board->base RMS high: re-run touch, keep gripper vertical")
    else:
        print("  board->base: MISSING")
    print(f"  square_length_mm in config = {config.BOARD.square_length_mm} "
          "(must be the measured value)")
    print("  ready for: locate.py --grab --json -> solve_topdown_ik")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", choices=["homography", "touch"], default=None,
                    help="run a single step (default: all)")
    ap.add_argument("--board-image", default=None, help="board frame file (skip grab)")
    ap.add_argument("--backend", default=None,
                    choices=[None, "soracom", "rtsp", "mcp"])
    ap.add_argument("--corner-ids", default=None,
                    help="explicit touch corner ids (else auto-picked in step 1)")
    ap.add_argument("--n", type=int, default=6, help="number of touch targets")
    ap.add_argument("--scale", action="store_true",
                    help="estimate board->base similarity scale (square unmeasured)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--yes", action="store_true", help="don't prompt before grabbing")
    args = ap.parse_args()

    corner_ids = [int(x) for x in args.corner_ids.split(",")] if args.corner_ids else None

    if args.only in (None, "homography"):
        res = step_homography(args)
        if corner_ids is None:
            corner_ids = res["touch_ids"]
    if args.only in (None, "touch"):
        if corner_ids is None:
            raise SystemExit("--only touch needs --corner-ids (or run homography first)")
        step_touch(args, corner_ids)
    step_summary()


if __name__ == "__main__":
    main()
