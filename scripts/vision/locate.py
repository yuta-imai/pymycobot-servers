#!/usr/bin/env python3
"""Step C: locate an object on the board -> base-frame grasp point (locate contract).

Homography pipeline (per pick, open-loop; matches docs/integration-plan.md):
  1. grab a frame
  2. segment the object by differencing against a once-captured EMPTY-board
     reference (static scene -> robust, no ML); largest blob = the object
  3. take the object's board-contact pixel (centroid; ~ the contact for the
     near-top-down mount, where out-of-plane parallax is small)
  4. pixel -> board (X,Y) via the calibrated homography
  5. board -> base via the touch-calibrated board->base transform
  6. grasp Z = board plane (from board->base) + config.GRASP.grasp_z_offset_mm
     (object height is a configured constant by design)

Output: the locate contract dict {ok, base_xyz_mm, yaw_deg, ...} ready for
solve_topdown_ik / pick_at. Requires OpenCV.

    python locate.py --save-ref --image empty_board.png   # once, board empty
    python locate.py --image with_object.png              # each pick
    python locate.py --grab --json                        # live, machine output
"""

from __future__ import annotations

import argparse
import json

import numpy as np

import config
from homography_calibrate import load_homography, fit_from_frame
from touch_calibrate import load_board_to_base
from geometry import apply_homography, base_grasp_point

REF_PATH = config.artifact_path("empty_board_ref.png")


def _largest_object_blob(gray, ref_gray, min_area_frac=0.002):
    import cv2
    diff = cv2.absdiff(gray, ref_gray)
    diff = cv2.GaussianBlur(diff, (5, 5), 0)
    _, mask = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area_frac * gray.shape[0] * gray.shape[1]:
        return None
    return c


def _centroid_pixel(contour):
    import cv2
    m = cv2.moments(contour)
    if abs(m["m00"]) < 1e-6:
        return contour.reshape(-1, 2).mean(0)
    return np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])


def _yaw_deg_base(contour, H_img2board, T_base_board):
    """Object long-axis as a base-frame yaw (deg): map the minAreaRect long axis
    through the homography to the board plane, then into base. Informational --
    solve_topdown_ik currently ignores yaw."""
    import cv2
    (cx, cy), (w, h), ang = cv2.minAreaRect(contour)
    long_ang = np.radians(ang if w >= h else ang + 90.0)
    L = max(w, h) / 2.0
    p0 = np.array([cx, cy])
    p1 = p0 + L * np.array([np.cos(long_ang), np.sin(long_ang)])
    b0, b1 = apply_homography(H_img2board, np.vstack([p0, p1]))
    v_board = np.array([b1[0] - b0[0], b1[1] - b0[1], 0.0])
    v_base = T_base_board[:3, :3] @ v_board
    return float(np.degrees(np.arctan2(v_base[1], v_base[0])))


def locate(frame, H_img2board, T_base_board, ref_gray, grasp_z_offset=None):
    import cv2
    grasp_z_offset = config.GRASP.grasp_z_offset_mm if grasp_z_offset is None \
        else grasp_z_offset
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    contour = _largest_object_blob(gray, ref_gray)
    if contour is None:
        return {"ok": False, "reason": "no_object_found"}

    pixel = _centroid_pixel(contour)
    bxy = apply_homography(H_img2board, np.asarray(pixel).reshape(1, 2))[0]
    foot, grasp = base_grasp_point(pixel, H_img2board, T_base_board, grasp_z_offset)
    yaw = _yaw_deg_base(contour, H_img2board, T_base_board)
    return {
        "ok": True,
        "base_xyz_mm": [round(float(v), 1) for v in grasp],
        "footprint_xyz_mm": [round(float(v), 1) for v in foot],
        "board_xy_mm": [round(float(bxy[0]), 1), round(float(bxy[1]), 1)],
        "yaw_deg": round(yaw, 1),
        "grasp_z_offset_mm": grasp_z_offset,
        "pixel": [round(float(pixel[0]), 1), round(float(pixel[1]), 1)],
    }


def main():
    import cv2
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", help="frame to process")
    src.add_argument("--grab", action="store_true")
    ap.add_argument("--save-ref", action="store_true",
                    help="save this (empty-board) frame as the reference and exit")
    ap.add_argument("--refresh", action="store_true",
                    help="re-fit the homography from this frame (drift robustness)")
    ap.add_argument("--url", default=None)
    ap.add_argument("--mcp-json", default=None)
    ap.add_argument("--json", action="store_true", help="print only the result JSON")
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"cannot read {args.image}")
    else:
        from frame_source import grab
        frame = grab(url=args.url, mcp_json=args.mcp_json)

    if args.save_ref:
        config.ensure_artifact_dir()
        cv2.imwrite(REF_PATH, frame)
        print(f"saved empty-board reference -> {REF_PATH}")
        return

    H = fit_from_frame(frame)["H_img2board"] if args.refresh \
        else load_homography()["H_img2board"]
    H = np.array(H, float)
    T_base_board = load_board_to_base()
    ref = cv2.imread(REF_PATH, cv2.IMREAD_GRAYSCALE)
    if ref is None:
        raise SystemExit(f"no empty-board reference at {REF_PATH}; run --save-ref")

    result = locate(frame, H, T_base_board, ref)
    print(json.dumps(result) if args.json else json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
