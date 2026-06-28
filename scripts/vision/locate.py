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
from geometry import apply_homography, base_grasp_point, board_to_base_fn

REF_PATH = config.artifact_path("empty_board_ref.png")


def board_region_mask(shape, H_img2board, inset_mm=8.0):
    """Binary mask of the board surface in the image (so detection ignores the
    moving background/person and anything off the board). Built by mapping the
    board's mm extent corners into the image via the inverse homography."""
    import cv2
    H_board2img = np.linalg.inv(np.asarray(H_img2board, float))
    w, h = config.BOARD.width_mm, config.BOARD.height_mm
    corners_mm = np.array([[inset_mm, inset_mm], [w - inset_mm, inset_mm],
                           [w - inset_mm, h - inset_mm], [inset_mm, h - inset_mm]],
                          float)
    poly = apply_homography(H_board2img, corners_mm).astype(np.int32)
    mask = np.zeros(shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, poly, 255)
    return mask


def _largest_object_blob(gray, ref_gray, region_mask=None, min_area_frac=0.002):
    import cv2
    diff = cv2.absdiff(gray, ref_gray)
    diff = cv2.GaussianBlur(diff, (5, 5), 0)
    _, mask = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if region_mask is not None:
        mask = cv2.bitwise_and(mask, region_mask)
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


def _yaw_deg_base(contour, H_img2board, H_board2base):
    """Object long-axis as a base-frame yaw (deg): map the minAreaRect long axis
    image->board->base (both homographies) and take the angle. Informational --
    solve_topdown_ik currently ignores yaw."""
    import cv2
    (cx, cy), (w, h), ang = cv2.minAreaRect(contour)
    long_ang = np.radians(ang if w >= h else ang + 90.0)
    L = max(w, h) / 2.0
    p0 = np.array([cx, cy])
    p1 = p0 + L * np.array([np.cos(long_ang), np.sin(long_ang)])
    b = apply_homography(H_img2board, np.vstack([p0, p1]))
    base = board_to_base_fn(H_board2base)(b)
    v = base[1] - base[0]
    return float(np.degrees(np.arctan2(v[1], v[0])))


def locate(frame, H_img2board, b2b, ref_gray, grasp_z_offset=None,
           min_area_frac=0.002):
    import cv2
    grasp_z_offset = config.GRASP.grasp_z_offset_mm if grasp_z_offset is None \
        else grasp_z_offset
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    region = board_region_mask(gray.shape, H_img2board)
    contour = _largest_object_blob(gray, ref_gray, region_mask=region,
                                   min_area_frac=min_area_frac)
    if contour is None:
        return {"ok": False, "reason": "no_object_found"}

    pixel = _centroid_pixel(contour)
    bxy = apply_homography(H_img2board, np.asarray(pixel).reshape(1, 2))[0]
    foot, grasp = base_grasp_point(pixel, H_img2board, b2b["H_board2base"],
                                   b2b["z_plane"], grasp_z_offset)
    yaw = _yaw_deg_base(contour, H_img2board, b2b["H_board2base"])
    return {
        "ok": True,
        "base_xyz_mm": [round(float(v), 1) for v in grasp],
        "footprint_xyz_mm": [round(float(v), 1) for v in foot],
        "board_xy_mm": [round(float(bxy[0]), 1), round(float(bxy[1]), 1)],
        "yaw_deg": round(yaw, 1),
        "grasp_z_offset_mm": grasp_z_offset,
        "pixel": [round(float(pixel[0]), 1), round(float(pixel[1]), 1)],
    }


def grasp_succeeded(before_xy_mm, after_frame, H_img2board, b2b, ref_gray,
                    tol_mm=20.0, min_area_frac=0.0008):
    """Vision-based grasp check (no gripper feedback on this unit).

    Call AFTER lift + return-to-home (so a grasped object leaves the board).
    Grasped == the object is gone from its pre-pick board location:
    locate finds nothing on the board, or the nearest blob moved > tol_mm.
    """
    res = locate(after_frame, H_img2board, b2b, ref_gray,
                 min_area_frac=min_area_frac)
    if not res.get("ok"):
        return {"grasped": True, "reason": "board_empty"}
    axy = res["board_xy_mm"]
    dist = ((axy[0] - before_xy_mm[0]) ** 2 + (axy[1] - before_xy_mm[1]) ** 2) ** 0.5
    return {"grasped": dist > tol_mm, "reason": "object_still_on_board",
            "after_board_xy_mm": axy, "moved_mm": round(float(dist), 1)}


def main():
    import cv2
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", help="frame to process")
    src.add_argument("--grab", action="store_true", help="grab via frame_source backend")
    src.add_argument("--mcp-json", default=None,
                     help="read a saved soracam MCP still result (JSON w/ imageBase64); "
                          "single-credential path via the soracam MCP, no .env")
    ap.add_argument("--save-ref", action="store_true",
                    help="save this (empty-board) frame as the reference and exit")
    ap.add_argument("--refresh", action="store_true",
                    help="re-fit the homography from this frame (drift robustness)")
    ap.add_argument("--url", default=None)
    ap.add_argument("--json", action="store_true", help="print only the result JSON")
    ap.add_argument("--save-annot", default=None,
                    help="save an annotated crop (board dimmed, detection marked)")
    ap.add_argument("--min-area-frac", type=float, default=0.002,
                    help="min object blob area as fraction of the image (lower for small objects)")
    ap.add_argument("--verify-removed", default=None, metavar="X,Y",
                    help="grasp check: report grasped=true if no object remains near "
                         "board (X,Y) mm (run after pick+home)")
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

    if args.save_ref:
        config.ensure_artifact_dir()
        cv2.imwrite(REF_PATH, frame)
        print(f"saved empty-board reference -> {REF_PATH}")
        return

    H = fit_from_frame(frame)["H_img2board"] if args.refresh \
        else load_homography()["H_img2board"]
    H = np.array(H, float)
    b2b = load_board_to_base()
    ref = cv2.imread(REF_PATH, cv2.IMREAD_GRAYSCALE)
    if ref is None:
        raise SystemExit(f"no empty-board reference at {REF_PATH}; run --save-ref")

    if args.verify_removed:
        bx, by = (float(v) for v in args.verify_removed.split(","))
        out = grasp_succeeded((bx, by), frame, H, b2b, ref,
                              min_area_frac=args.min_area_frac)
        print(json.dumps(out))
        return

    result = locate(frame, H, b2b, ref, min_area_frac=args.min_area_frac)
    if args.save_annot and result.get("ok"):
        vis = frame.copy()
        region = board_region_mask(frame.shape, H)
        vis[region == 0] = (vis[region == 0] * 0.4).astype("uint8")
        px = tuple(int(v) for v in result["pixel"])
        cv2.circle(vis, px, 14, (0, 0, 255), 3)
        cv2.putText(vis, f"board{result['board_xy_mm']}", (px[0] + 12, px[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        r = 220
        crop = vis[max(0, px[1] - r):px[1] + r, max(0, px[0] - r):px[0] + r]
        cv2.imwrite(args.save_annot, crop)
    print(json.dumps(result) if args.json else json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
