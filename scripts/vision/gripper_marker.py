#!/usr/bin/env python3
"""Detect the gripper ArUco marker and report its board position (brain-side).

The marker (config.GRIPPER_MARKER) is stuck flat on the gripper TOP face,
centered on the approach axis. Detected full-frame, its center is mapped through
the pixel->board homography (calib/homography.json) to a board (x,y) in mm =
the gripper-tip board position. This is the closed-loop feedback for visual
servoing: it sidesteps the corrected_fk absolute-position distortion entirely.

Detector params are deliberately loose: the marker is small, can be blurry, and
is only frontal when the gripper is in a top-down pose. Detection is much more
reliable with the room lit and the gripper top face pointing up.

    python gripper_marker.py --mcp-json <saved-still>   # or --image f.jpg / --grab
    python gripper_marker.py --image f.jpg --save-annot /tmp/gm_annot.png
"""

from __future__ import annotations

import argparse
import json
from typing import Optional, Tuple

import numpy as np

import config
from homography_calibrate import load_homography


def _detector():
    import cv2
    par = cv2.aruco.DetectorParameters()
    par.adaptiveThreshWinSizeMin = 3
    par.adaptiveThreshWinSizeMax = 151
    par.adaptiveThreshWinSizeStep = 6
    par.minMarkerPerimeterRate = 0.008
    par.maxMarkerPerimeterRate = 4.0
    par.polygonalApproxAccuracyRate = 0.06
    par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    dic = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, config.GRIPPER_MARKER.dictionary))
    return cv2.aruco.ArucoDetector(dic, par)


def _glare_robust_variants(frame):
    """Gray variants that recover a glossy/glary marker (specular is blue-ish).

    Tried in order until the marker is found: plain gray, gamma-down (pull back
    highlights), per-pixel min over RGB (drops the bluish specular), and an
    illumination-flattened (background-division) gray.
    """
    import cv2
    if frame.ndim == 2:
        return [frame]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    b, g, r = cv2.split(frame)
    gamma = np.clip(((gray / 255.0) ** 1.6) * 255, 0, 255).astype(np.uint8)
    minrgb = np.minimum(np.minimum(r, g), b)
    bg = cv2.GaussianBlur(gray, (0, 0), 25)
    flat = cv2.normalize(gray.astype(np.float32) / (bg.astype(np.float32) + 1),
                         None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return [gray, gamma, minrgb, flat]


def detect_marker_pixel(frame) -> Optional[np.ndarray]:
    """Return the gripper-marker center pixel [x,y], or None if not detected.

    Restricts to the configured marker id (other ArUco hits in the scene are
    ignored). If the same id appears more than once, the largest is kept. Tries
    several glare-robust gray variants before giving up.
    """
    import cv2
    det = _detector()
    want = config.GRIPPER_MARKER.marker_id
    for gray in _glare_robust_variants(frame):
        corners, ids, _ = det.detectMarkers(gray)
        if ids is None:
            continue
        ids = ids.ravel()
        best, best_area = None, -1.0
        for c, i in zip(corners, ids):
            if int(i) != want:
                continue
            quad = c.reshape(-1, 2)
            area = cv2.contourArea(quad.astype(np.float32))
            if area > best_area:
                best, best_area = quad.mean(axis=0), area
        if best is not None:
            return best.astype(float)
    return None


def marker_board_xy(frame, H_img2board=None) -> Optional[Tuple[float, float]]:
    """Detect the marker and map its center pixel to board (x,y) in mm."""
    from geometry import apply_homography
    if H_img2board is None:
        H_img2board = load_homography()["H_img2board"]
    px = detect_marker_pixel(frame)
    if px is None:
        return None
    bd = apply_homography(np.asarray(H_img2board), px[None, :])[0]
    return float(bd[0]), float(bd[1])


def _load_frame(args):
    import cv2
    if args.image:
        return cv2.imread(args.image)
    if args.mcp_json:
        from frame_source import from_mcp_still
        return from_mcp_still(args.mcp_json)
    if args.grab:
        from frame_source import grab
        return grab()
    raise SystemExit("provide --image, --mcp-json, or --grab")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image")
    ap.add_argument("--mcp-json")
    ap.add_argument("--grab", action="store_true")
    ap.add_argument("--save-annot")
    args = ap.parse_args()
    import cv2
    frame = _load_frame(args)
    px = detect_marker_pixel(frame)
    if px is None:
        print(json.dumps({"detected": False}))
        raise SystemExit(1)
    H_img2board = load_homography()["H_img2board"]
    from geometry import apply_homography
    bd = apply_homography(np.asarray(H_img2board), px[None, :])[0]
    print(json.dumps({"detected": True,
                      "pixel": [round(float(px[0]), 1), round(float(px[1]), 1)],
                      "board_mm": [round(float(bd[0]), 1), round(float(bd[1]), 1)]}))
    if args.save_annot:
        out = frame.copy()
        cv2.circle(out, tuple(px.astype(int)), 14, (0, 0, 255), 3)
        cv2.putText(out, f"({bd[0]:.0f},{bd[1]:.0f})mm",
                    tuple((px + np.array([18, 0])).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.imwrite(args.save_annot, out)
        print(f"wrote {args.save_annot}")


if __name__ == "__main__":
    main()
