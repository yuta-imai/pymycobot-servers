#!/usr/bin/env python3
"""Step 2a: camera intrinsics + lens distortion from ChArUco views.

The SORACAM is fixed, so we get many board POSES by moving the A3 board around in
front of it (vary distance, tilt, position; keep it sharp and fill the frame).
Capture ~20-30 frames (frame_source.py --burst) into a folder, then:

    python calibrate_intrinsics.py --images captures/ --model rational

Writes config.calib/intrinsics.json = {model, image_size, K, dist, rms_px}.
The wide ATOM lens (~120deg) is handled by the 8-coefficient RATIONAL model by
default; pass --model fisheye for an equidistant (true fisheye) fit if the
rational reprojection RMS stays high (> ~1 px). `locate.py` reads `model` and
undistorts with the matching routine, so the chain stays consistent.

Requires OpenCV with aruco.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
from typing import List, Tuple

import numpy as np

import config
from charuco_board import build_board, chessboard_corners_mm, make_detector


def _gather(images_dir: str) -> List[str]:
    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp")
    files: List[str] = []
    for e in exts:
        files += glob.glob(os.path.join(images_dir, e))
    return sorted(files)


def _collect_points(files: List[str]):
    """Detect ChArUco corners in each image -> matched (objpoints, imgpoints)."""
    import cv2
    _, board = build_board()
    detect = make_detector(board)
    corners_mm = chessboard_corners_mm(board)        # (N,3) board-frame, mm

    objpoints, imgpoints = [], []
    image_size = None
    used = 0
    for f in files:
        img = cv2.imread(f)
        if img is None:
            print(f"[skip] unreadable: {f}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        ch_corners, ch_ids = detect(gray)
        if ch_ids is None:
            print(f"[skip] no/low ChArUco: {os.path.basename(f)}")
            continue
        ids = ch_ids.reshape(-1)
        objp = corners_mm[ids].astype(np.float32)
        imgp = ch_corners.reshape(-1, 2).astype(np.float32)
        objpoints.append(objp)
        imgpoints.append(imgp)
        used += 1
        print(f"[ok]  {os.path.basename(f)}: {len(ids)} corners")
    print(f"[collect] used {used}/{len(files)} images")
    return objpoints, imgpoints, image_size


def calibrate(images_dir: str, model: str = "rational") -> dict:
    import cv2
    files = _gather(images_dir)
    if not files:
        raise RuntimeError(f"no images in {images_dir}")
    objpoints, imgpoints, image_size = _collect_points(files)
    if len(objpoints) < 6:
        raise RuntimeError(f"need >= 6 good views, got {len(objpoints)}")

    if model == "rational":
        flags = cv2.CALIB_RATIONAL_MODEL
        rms, K, dist, _, _ = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, None, None, flags=flags)
        dist = dist.reshape(-1)
    elif model == "fisheye":
        # fisheye wants (M,1,3)/(M,1,2) float32 per view
        objp_f = [o.reshape(-1, 1, 3).astype(np.float64) for o in objpoints]
        imgp_f = [i.reshape(-1, 1, 2).astype(np.float64) for i in imgpoints]
        K = np.zeros((3, 3))
        dist = np.zeros((4, 1))
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                 | cv2.fisheye.CALIB_FIX_SKEW)
        rms, K, dist, _, _ = cv2.fisheye.calibrate(
            objp_f, imgp_f, image_size, K, dist, flags=flags)
        dist = dist.reshape(-1)
    else:
        raise ValueError("model must be 'rational' or 'fisheye'")

    out = {
        "model": model,
        "image_size": list(image_size),
        "K": np.asarray(K).tolist(),
        "dist": np.asarray(dist).tolist(),
        "rms_px": float(rms),
        "n_views": len(objpoints),
    }
    return out


# --- consumed by extrinsics / locate ---------------------------------------- #
def load_intrinsics(path: str = None) -> dict:
    path = path or config.artifact_path(config.INTRINSICS_JSON)
    with open(path) as f:
        d = json.load(f)
    d["K"] = np.array(d["K"], float)
    d["dist"] = np.array(d["dist"], float)
    return d


def undistort_to_normalized(pixels: np.ndarray, intr: dict) -> np.ndarray:
    """(N,2) pixels -> (N,2) undistorted normalized image coords, model-aware.

    Normalized coords feed geometry.normalized_pixel_to_ray.
    """
    import cv2
    pts = np.asarray(pixels, float).reshape(-1, 1, 2)
    K, dist = intr["K"], intr["dist"]
    if intr["model"] == "fisheye":
        out = cv2.fisheye.undistortPoints(pts, K, dist)
    else:
        out = cv2.undistortPoints(pts, K, dist)      # P=None -> normalized
    return out.reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True, help="folder of ChArUco captures")
    ap.add_argument("--model", default="rational", choices=["rational", "fisheye"])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    result = calibrate(args.images, args.model)
    config.ensure_artifact_dir()
    out = args.out or config.artifact_path(config.INTRINSICS_JSON)
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nmodel={result['model']}  views={result['n_views']}  "
          f"image={result['image_size']}  RMS={result['rms_px']:.3f} px")
    print(f"wrote {out}")
    if result["rms_px"] > 1.0:
        print("WARN: RMS > 1 px. Add sharper/closer/more-tilted views, or try "
              "--model fisheye.")


if __name__ == "__main__":
    main()
