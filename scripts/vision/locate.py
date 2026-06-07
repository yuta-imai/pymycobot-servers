#!/usr/bin/env python3
"""Step 3: locate an object on the board -> base-frame grasp point (locate contract).

Pipeline (per pick, open-loop, matches docs/integration-plan.md `locate`):
  1. grab a frame
  2. (optional) re-PnP the board to refresh camera->board drift; the board stays
     as the work surface, so it is visible around the object (ChArUco tolerates
     the partial occlusion)
  3. segment the object by differencing against a once-captured EMPTY-board
     reference (static scene -> robust, no ML); largest blob = the object
  4. take the object's board-CONTACT pixel (default) -- not the visual centroid,
     which back-projects behind the base for tall objects under an oblique view
  5. undistort -> camera ray -> intersect board plane -> board xy -> base xy
  6. grasp Z = board plane + config.GRASP.grasp_z_offset_mm (height is a
     configured constant per object class, by design -- we do not see height)

Output: the locate contract dict {ok, base_xyz_mm, yaw_deg, ...} ready to feed
solve_topdown_ik / pick_at. Requires OpenCV.

    python locate.py --save-ref --image empty_board.png     # once, board empty
    python locate.py --image with_object.png                # each pick
    python locate.py --grab --json                          # live, machine output
"""

from __future__ import annotations

import argparse
import json

import numpy as np

import config
from charuco_board import build_board, chessboard_corners_mm, make_detector
from calibrate_intrinsics import load_intrinsics, undistort_to_normalized
from calibrate_extrinsics import load_extrinsics, solve as solve_extrinsics
from touch_calibrate import load_board_to_base
from geometry import (normalized_pixel_to_ray, base_grasp_point,
                      transform_points, invert_transform)

REF_PATH = config.artifact_path("empty_board_ref.png")


def _largest_object_blob(gray, ref_gray, min_area_frac=0.002):
    """Diff vs empty-board reference -> largest connected object region.

    Returns (contour, mask) or (None, None). min_area_frac is of the image area.
    """
    import cv2
    diff = cv2.absdiff(gray, ref_gray)
    diff = cv2.GaussianBlur(diff, (5, 5), 0)
    _, mask = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, mask
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area_frac * gray.shape[0] * gray.shape[1]:
        return None, mask
    return c, mask


def _contact_pixel(contour, nadir_px):
    """Object pixel where it meets the board: the contour point nearest the image
    of the board point directly under the camera (nadir). For a top-down mount
    this ~ the centroid; for an oblique mount it picks the base edge, killing the
    centroid-vs-contact parallax."""
    pts = contour.reshape(-1, 2).astype(float)
    d = np.linalg.norm(pts - np.asarray(nadir_px, float), axis=1)
    return pts[int(np.argmin(d))]


def _centroid_pixel(contour):
    import cv2
    m = cv2.moments(contour)
    if abs(m["m00"]) < 1e-6:
        return contour.reshape(-1, 2).mean(0)
    return np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])


def _nadir_pixel(T_cam_board, intr):
    """Pixel of the board point directly below the camera (foot of the camera
    normal onto the board plane) -- the 'most top-down' direction in the image."""
    import cv2
    # camera centre in board frame:
    cam_in_board = transform_points(invert_transform(T_cam_board), np.zeros(3))
    nadir_board = np.array([cam_in_board[0], cam_in_board[1], 0.0])  # drop to plane
    nadir_cam = transform_points(T_cam_board, nadir_board)
    rvec = np.zeros(3); tvec = np.zeros(3)
    if intr["model"] == "fisheye":
        px, _ = cv2.fisheye.projectPoints(nadir_cam.reshape(-1, 1, 3),
                                          rvec, tvec, intr["K"], intr["dist"])
    else:
        px, _ = cv2.projectPoints(nadir_cam.reshape(-1, 1, 3),
                                  rvec, tvec, intr["K"], intr["dist"])
    return px.reshape(2)


def _yaw_deg_base(contour, intr, T_cam_board, T_base_board):
    """Object long-axis orientation as a base-frame yaw (deg). Back-projects the
    minAreaRect long axis onto the board plane, then into base. Informational:
    solve_topdown_ik currently ignores yaw."""
    import cv2
    from geometry import board_point_from_ray
    rect = cv2.minAreaRect(contour)
    (cx, cy), (w, h), ang = rect
    # endpoint of the long axis from the rect centre
    long_ang = np.radians(ang if w >= h else ang + 90.0)
    L = max(w, h) / 2.0
    p0 = np.array([cx, cy])
    p1 = p0 + L * np.array([np.cos(long_ang), np.sin(long_ang)])
    bpts = []
    for p in (p0, p1):
        n = undistort_to_normalized(p.reshape(1, 2), intr)[0]
        ray = normalized_pixel_to_ray(n[0], n[1])
        bpts.append(board_point_from_ray(ray, T_cam_board))
    v_board = np.array(bpts[1]) - np.array(bpts[0])
    v_base = T_base_board[:3, :3] @ v_board
    return float(np.degrees(np.arctan2(v_base[1], v_base[0])))


def locate(frame, intr, T_cam_board, T_base_board, ref_gray,
           use_contact=True, grasp_z_offset=None):
    import cv2
    grasp_z_offset = config.GRASP.grasp_z_offset_mm if grasp_z_offset is None \
        else grasp_z_offset
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    contour, _ = _largest_object_blob(gray, ref_gray)
    if contour is None:
        return {"ok": False, "reason": "no_object_found"}

    if use_contact:
        nadir = _nadir_pixel(T_cam_board, intr)
        pixel = _contact_pixel(contour, nadir)
    else:
        pixel = _centroid_pixel(contour)

    n = undistort_to_normalized(np.asarray(pixel).reshape(1, 2), intr)[0]
    ray = normalized_pixel_to_ray(n[0], n[1])
    try:
        foot, grasp = base_grasp_point(ray, T_cam_board, T_base_board,
                                       grasp_z_offset)
    except ValueError as e:
        return {"ok": False, "reason": f"ray_plane_failed: {e}"}

    yaw = _yaw_deg_base(contour, intr, T_cam_board, T_base_board)
    return {
        "ok": True,
        "base_xyz_mm": [round(float(v), 1) for v in grasp],
        "footprint_xyz_mm": [round(float(v), 1) for v in foot],
        "yaw_deg": round(yaw, 1),
        "grasp_z_offset_mm": grasp_z_offset,
        "pixel": [round(float(pixel[0]), 1), round(float(pixel[1]), 1)],
        "method": "contact" if use_contact else "centroid",
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
    ap.add_argument("--refresh-pnp", action="store_true",
                    help="re-solve camera->board from this frame (drift robustness)")
    ap.add_argument("--centroid", action="store_true",
                    help="use visual centroid instead of board-contact point")
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

    intr = load_intrinsics()
    if args.refresh_pnp:
        ext = solve_extrinsics(frame, intr)
        T_cam_board = np.array(ext["T_cam_board"], float)
    else:
        T_cam_board = load_extrinsics()
    T_base_board = load_board_to_base()
    ref = cv2.imread(REF_PATH, cv2.IMREAD_GRAYSCALE)
    if ref is None:
        raise SystemExit(f"no empty-board reference at {REF_PATH}; run --save-ref")

    result = locate(frame, intr, T_cam_board, T_base_board, ref,
                    use_contact=not args.centroid)
    if args.json:
        print(json.dumps(result))
    else:
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
