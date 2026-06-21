#!/usr/bin/env python3
"""Step A: pixel <-> board-plane homography from ONE board view.

The fixed camera + planar work surface let us skip lens intrinsics: detect the
A4 ChArUco corners in a single frame and fit a planar homography mapping image
pixels <-> board-plane mm. Lens distortion is absorbed within the board region
(where objects are placed), so no multi-view intrinsics calibration is needed.

    python homography_calibrate.py --image board_clear.png   # board fully visible
    python homography_calibrate.py --grab                     # via frame_source

Capture with the ARM CLEAR of the board so corners aren't occluded. Writes
config.calib/homography.json (H both directions, leave-one-out residual mm,
corner count). The board stays as the work surface afterward. Requires OpenCV.
"""

from __future__ import annotations

import argparse
import json

import numpy as np

import config
from charuco_board import build_board, make_detector, chessboard_corners_mm


def fit_from_frame(frame) -> dict:
    import cv2
    _, board = build_board()
    detect = make_detector(board)
    corners_mm = chessboard_corners_mm(board)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cc, ci = detect(gray)
    if ci is None or len(ci) < 8:
        raise RuntimeError(f"need >= 8 ChArUco corners, got "
                           f"{0 if ci is None else len(ci)} (arm clear? in focus?)")
    ids = ci.flatten()
    px = cc.reshape(-1, 2).astype(np.float64)
    bd = corners_mm[ids][:, :2].astype(np.float64)

    H_i2b, _ = cv2.findHomography(px, bd, 0)         # least squares
    H_b2i, _ = cv2.findHomography(bd, px, 0)

    # leave-one-out residual in board mm = honest accuracy estimate
    loo = []
    for k in range(len(ids)):
        m = np.ones(len(ids), bool); m[k] = False
        Hk, _ = cv2.findHomography(px[m], bd[m], 0)
        p = cv2.perspectiveTransform(px[k].reshape(1, 1, 2), Hk).reshape(2)
        loo.append(float(np.linalg.norm(p - bd[k])))
    loo = np.array(loo)

    return {
        "H_img2board": H_i2b.tolist(),
        "H_board2img": H_b2i.tolist(),
        "n_corners": int(len(ids)),
        "image_size": [int(gray.shape[1]), int(gray.shape[0])],
        "square_length_mm": config.BOARD.square_length_mm,
        "loo_residual_mm": {"mean": float(loo.mean()),
                            "p95": float(np.percentile(loo, 95)),
                            "max": float(loo.max())},
    }


def load_homography(path: str = None) -> dict:
    path = path or config.artifact_path(config.HOMOGRAPHY_JSON)
    with open(path) as f:
        d = json.load(f)
    d["H_img2board"] = np.array(d["H_img2board"], float)
    d["H_board2img"] = np.array(d["H_board2img"], float)
    return d


def main():
    import cv2
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", help="frame with the board fully visible")
    src.add_argument("--grab", action="store_true", help="grab via frame_source backend")
    src.add_argument("--mcp-json", default=None,
                     help="read a saved soracam MCP still result (single-credential path)")
    ap.add_argument("--url", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"cannot read {args.image}")
    elif args.mcp_json:
        from frame_source import from_mcp_still
        frame = from_mcp_still(args.mcp_json)
    else:
        from frame_source import grab
        frame = grab(url=args.url)

    result = fit_from_frame(frame)
    config.ensure_artifact_dir()
    out = args.out or config.artifact_path(config.HOMOGRAPHY_JSON)
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    r = result["loo_residual_mm"]
    print(f"corners={result['n_corners']}  square={result['square_length_mm']}mm")
    print(f"LOO residual: mean={r['mean']:.2f}mm  p95={r['p95']:.2f}mm  max={r['max']:.2f}mm")
    print(f"wrote {out}")
    if r["mean"] > 2.0:
        print("WARN: residual high — recapture with the board flat, fully visible, "
              "well-lit, and filling more of the frame.")


if __name__ == "__main__":
    main()
