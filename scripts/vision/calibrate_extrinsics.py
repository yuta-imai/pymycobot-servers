#!/usr/bin/env python3
"""Step 2b: single-shot camera->board pose (PnP).

With the A3 board lying flat in the work position, detect the ChArUco corners in
one frame and solve the board pose in the camera frame. This fixes the board
PLANE in camera coordinates, which `locate.py` intersects pixel rays against.

    python calibrate_extrinsics.py --image board_flat.png
    python calibrate_extrinsics.py --grab            # pull a frame via frame_source

Writes config.calib/extrinsics.json = {T_cam_board (4x4), reproj_rms_px}.
Because the board stays as the work surface, you can also re-run this (or re-PnP
inside locate) any time to absorb camera drift.

Distortion is handled by undistorting corners to NORMALIZED coords (using the
intrinsics model, rational or fisheye) and solving PnP against an identity
camera, so both models share one code path. Requires OpenCV with aruco.
"""

from __future__ import annotations

import argparse
import json

import numpy as np

import config
from charuco_board import build_board, chessboard_corners_mm, make_detector
from calibrate_intrinsics import load_intrinsics, undistort_to_normalized
from geometry import make_transform


def solve(image, intr: dict) -> dict:
    import cv2
    _, board = build_board()
    detect = make_detector(board)
    corners_mm = chessboard_corners_mm(board)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ch_corners, ch_ids = detect(gray)
    if ch_ids is None:
        raise RuntimeError("no ChArUco board detected in the frame")
    ids = ch_ids.reshape(-1)
    objp = corners_mm[ids].astype(np.float64)
    imgp = ch_corners.reshape(-1, 2).astype(np.float64)

    # undistort -> normalized, then PnP against identity intrinsics
    norm = undistort_to_normalized(imgp, intr).astype(np.float64)
    eye = np.eye(3)
    zero = np.zeros(5)
    ok, rvec, tvec = cv2.solvePnP(objp, norm, eye, zero,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")

    R, _ = cv2.Rodrigues(rvec)
    T_cam_board = make_transform(R, tvec.reshape(3))

    # reprojection RMS (in pixels, via the real intrinsics) for a quality gate
    proj, _ = cv2.projectPoints(objp, rvec, tvec, intr["K"], intr["dist"]) \
        if intr["model"] != "fisheye" else \
        cv2.fisheye.projectPoints(objp.reshape(-1, 1, 3), rvec, tvec,
                                  intr["K"], intr["dist"])
    proj = proj.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((proj - imgp) ** 2, axis=1))))

    return {
        "T_cam_board": T_cam_board.tolist(),
        "reproj_rms_px": rms,
        "n_corners": int(len(ids)),
    }


def load_extrinsics(path: str = None) -> np.ndarray:
    path = path or config.artifact_path(config.EXTRINSICS_JSON)
    with open(path) as f:
        d = json.load(f)
    return np.array(d["T_cam_board"], float)


def main():
    import cv2
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", help="frame with the board flat in work position")
    src.add_argument("--grab", action="store_true", help="grab via frame_source")
    ap.add_argument("--url", default=None)
    ap.add_argument("--mcp-json", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise SystemExit(f"cannot read {args.image}")
    else:
        from frame_source import grab
        img = grab(url=args.url, mcp_json=args.mcp_json)

    intr = load_intrinsics()
    result = solve(img, intr)
    config.ensure_artifact_dir()
    out = args.out or config.artifact_path(config.EXTRINSICS_JSON)
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    T = np.array(result["T_cam_board"])
    print(f"corners={result['n_corners']}  reproj_rms={result['reproj_rms_px']:.3f} px")
    print(f"board origin in camera (mm): {T[:3, 3].round(1)}")
    print(f"camera height above board ~ {abs(T[2, 3]):.0f} mm (board z in cam)")
    print(f"wrote {out}")
    if result["reproj_rms_px"] > 1.5:
        print("WARN: high reprojection RMS — re-check focus/intrinsics.")


if __name__ == "__main__":
    main()
