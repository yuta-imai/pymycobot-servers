#!/usr/bin/env python3
"""Render a single ArUco marker to a true-scale A4 PDF (for the gripper marker).

Prints at exactly --mm so the gripper marker is the real size the detector expects.
Reuses board_to_pdf.write_pdf (lossless, no third-party deps). Includes a 50 mm
verification scale bar and placement instructions.

    python marker_to_pdf.py --dict DICT_5X5_100 --id 0 --mm 30 --out gripper_marker_A4.pdf

Print at 100% scale (no fit-to-page), measure the 50 mm bar, then stick it flat on
the gripper TOP face, centered on the approach (tip) axis, facing the camera.
"""

from __future__ import annotations

import argparse

import numpy as np

from board_to_pdf import write_pdf

A4_MM = (210.0, 297.0)


def render(dict_name: str, marker_id: int, mm: float, dpi: int,
           quiet_mm: float = 6.0) -> np.ndarray:
    import cv2
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    mm2px = dpi / 25.4
    mpx = int(round(mm * mm2px))
    try:
        marker = aruco.generateImageMarker(dictionary, marker_id, mpx)
    except AttributeError:
        marker = aruco.drawMarker(dictionary, marker_id, mpx)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    pw = int(round(A4_MM[0] * mm2px))
    ph = int(round(A4_MM[1] * mm2px))
    page = np.full((ph, pw, 3), 255, np.uint8)
    q = int(round(quiet_mm * mm2px))            # white quiet zone around the marker

    ox = (pw - mpx) // 2
    oy = int(round(60 * mm2px))                 # marker block near the top
    # white quiet border is already white page; just place the marker
    page[oy:oy + mpx, ox:ox + mpx] = marker

    black = (0, 0, 0)
    th = max(1, int(round(mm2px / 12)))
    FONT = cv2.FONT_HERSHEY_SIMPLEX

    def text(s, x, y, scale):
        cv2.putText(page, s, (x, y), FONT, scale, black, th, cv2.LINE_AA)

    text(f"{dict_name} id={marker_id}  {mm:.0f} mm  -- print at 100% scale",
         int(20 * mm2px), int(30 * mm2px), mm2px / 8)
    text("Stick flat on the gripper TOP face, centered on the tip (approach) axis.",
         int(20 * mm2px), int(42 * mm2px), mm2px / 11)

    # crop ticks at the marker corners
    t = int(round(4 * mm2px))
    for cx, cy in [(ox, oy), (ox + mpx, oy), (ox, oy + mpx), (ox + mpx, oy + mpx)]:
        cv2.line(page, (cx - t, cy), (cx + t, cy), black, th)
        cv2.line(page, (cx, cy - t), (cx, cy + t), black, th)

    # 50 mm verification scale bar below the marker
    by = oy + mpx + int(round(25 * mm2px))
    bx = ox
    bx2 = bx + int(round(50 * mm2px))
    cap = int(round(2 * mm2px))
    cv2.line(page, (bx, by), (bx2, by), black, th)
    cv2.line(page, (bx, by - cap), (bx, by + cap), black, th)
    cv2.line(page, (bx2, by - cap), (bx2, by + cap), black, th)
    text("50 mm (measure to verify 100% print)", bx, by + int(8 * mm2px), mm2px / 9)
    return page


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dict", default="DICT_5X5_100")
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--mm", type=float, default=30.0)
    ap.add_argument("--dpi", type=int, default=600)
    ap.add_argument("--out", default="gripper_marker_A4.pdf")
    args = ap.parse_args()
    page = render(args.dict, args.id, args.mm, args.dpi)
    path = write_pdf(args.out, page, A4_MM)
    import os
    print(f"wrote {path}  ({os.path.getsize(path)//1024} KB)")
    print(f"  {args.dict} id={args.id}  {args.mm:.0f} mm on A4; print at 100% scale.")


if __name__ == "__main__":
    main()
