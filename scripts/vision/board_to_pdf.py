#!/usr/bin/env python3
"""Render a ChArUco board to a print-ready, true-scale PDF (A4 by default).

Why PDF: the page is sized in real millimetres, so printing at 100% scale
(no "fit to page") reproduces the squares at exactly square_length_mm -- which is
what makes the metric pick coordinates correct. A 50 mm verification scale bar is
printed on the page so you can confirm the print scale with a ruler.

No third-party deps: the board is drawn with OpenCV and embedded losslessly
(FlateDecode) into a hand-written PDF, so it never adds JPEG artifacts that could
hurt detection of the printed board.

Defaults come from config.BOARD. Override for a detection-friendly reprint, e.g.
a coarser dictionary with bigger markers (more px/module -> robust decoding):

    python board_to_pdf.py --out charuco_A4.pdf
    python board_to_pdf.py --dict DICT_4X4_50 --squares 10x7 \
        --square-mm 28 --marker-mm 22 --out charuco_4x4_A4.pdf

Then print at 100% scale, measure the 50 mm bar (and a square), and set
config.BOARD.square_length_mm to the measured square edge.
"""

from __future__ import annotations

import argparse
import zlib

import numpy as np

from config import BOARD

A4_MM = (210.0, 297.0)            # (width, height) portrait


# --------------------------------------------------------------------------- #
# Board image
# --------------------------------------------------------------------------- #
def build_board(dict_name, squares_x, squares_y, square_mm, marker_mm):
    import cv2
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    try:
        board = aruco.CharucoBoard((squares_x, squares_y), square_mm, marker_mm,
                                   dictionary)
    except AttributeError:
        board = aruco.CharucoBoard_create(squares_x, squares_y, square_mm,
                                          marker_mm, dictionary)
    return board


def render_page(args) -> np.ndarray:
    """Compose the full A4 page (white) with the board placed at exact mm and a
    title + 50 mm scale bar + corner crop ticks, all in black. Returns BGR uint8.
    """
    import cv2
    sx, sy = (int(v) for v in args.squares.lower().split("x"))
    dpi = args.dpi
    mm2px = dpi / 25.4

    board = build_board(args.dict, sx, sy, args.square_mm, args.marker_mm)
    bw_px = int(round(sx * args.square_mm * mm2px))
    bh_px = int(round(sy * args.square_mm * mm2px))
    try:
        board_img = board.generateImage((bw_px, bh_px), marginSize=0)
    except AttributeError:
        board_img = board.draw((bw_px, bh_px), marginSize=0)
    board_img = cv2.cvtColor(board_img, cv2.COLOR_GRAY2BGR)

    board_w_mm, board_h_mm = sx * args.square_mm, sy * args.square_mm
    # orientation: landscape if the board is wider than tall
    page_mm = A4_MM if board_h_mm >= board_w_mm else (A4_MM[1], A4_MM[0])
    pw = int(round(page_mm[0] * mm2px))
    ph = int(round(page_mm[1] * mm2px))
    if board_w_mm > page_mm[0] or board_h_mm > page_mm[1]:
        raise SystemExit(f"board {board_w_mm:.0f}x{board_h_mm:.0f}mm does not fit "
                         f"page {page_mm[0]:.0f}x{page_mm[1]:.0f}mm")

    page = np.full((ph, pw, 3), 255, np.uint8)
    # center the board; annotations go in the page margins
    ox = (pw - bw_px) // 2
    oy = (ph - bh_px) // 2
    page[oy:oy + bh_px, ox:ox + bw_px] = board_img

    black = (0, 0, 0)
    th = max(1, int(round(mm2px / 12)))
    FONT = cv2.FONT_HERSHEY_SIMPLEX

    def font_for(text, target_w, max_h):
        (tw, h), _ = cv2.getTextSize(text, FONT, 1.0, th)
        return min(target_w / max(tw, 1), max_h / max(h, 1))

    # title in the top margin, auto-fit to board width / available band
    title = (f"ChArUco {sx}x{sy} {args.dict} square={args.square_mm}mm "
             f"marker={args.marker_mm}mm - print at 100%")
    if oy > int(3 * mm2px):
        fs = min(font_for(title, bw_px, oy - int(2 * mm2px)), mm2px / 6)
        (tw, tht), _ = cv2.getTextSize(title, FONT, fs, th)
        cv2.putText(page, title, (ox, max(tht + 4, (oy - tht) // 2 + tht)),
                    FONT, fs, black, th, cv2.LINE_AA)

    # corner crop ticks around the board
    t = int(round(4 * mm2px))
    for cx, cy in [(ox, oy), (ox + bw_px, oy), (ox, oy + bh_px),
                   (ox + bw_px, oy + bh_px)]:
        cv2.line(page, (cx - t, cy), (cx + t, cy), black, th)
        cv2.line(page, (cx, cy - t), (cx, cy + t), black, th)

    # 50 mm verification scale bar in the bottom margin (board nearly fills A4,
    # so the band is small -> keep the bar thin with a compact label)
    bottom = ph - (oy + bh_px)
    if bottom > int(4 * mm2px):
        bar_mm = 50.0
        bx = ox
        by = oy + bh_px + min(bottom // 2, int(5 * mm2px))
        bx2 = bx + int(round(bar_mm * mm2px))
        cap = int(round(1.5 * mm2px))
        cv2.line(page, (bx, by), (bx2, by), black, th)
        cv2.line(page, (bx, by - cap), (bx, by + cap), black, th)
        cv2.line(page, (bx2, by - cap), (bx2, by + cap), black, th)
        lbl = "50 mm (measure to verify 100% print)"
        fs = min(font_for(lbl, bw_px - int(round(bar_mm * mm2px)) - int(2 * mm2px),
                          int(3 * mm2px)), mm2px / 7)
        cv2.putText(page, lbl, (bx2 + int(2 * mm2px), by + int(mm2px)),
                    FONT, fs, black, th, cv2.LINE_AA)

    return page, page_mm


# --------------------------------------------------------------------------- #
# Minimal PDF writer (single full-page lossless RGB image)
# --------------------------------------------------------------------------- #
def write_pdf(path: str, page_bgr: np.ndarray, page_mm) -> str:
    import cv2
    h, w = page_bgr.shape[:2]
    rgb = cv2.cvtColor(page_bgr, cv2.COLOR_BGR2RGB)
    raw = zlib.compress(rgb.tobytes(), 9)
    pw_pt = page_mm[0] * 72.0 / 25.4
    ph_pt = page_mm[1] * 72.0 / 25.4

    objs = []
    objs.append(b"<< /Type /Catalog /Pages 2 0 R >>")
    objs.append(b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>")
    objs.append((f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {pw_pt:.3f} "
                 f"{ph_pt:.3f}] /Resources << /XObject << /Im0 4 0 R >> >> "
                 f"/Contents 5 0 R >>").encode())
    img_hdr = (f"<< /Type /XObject /Subtype /Image /Width {w} /Height {h} "
               f"/ColorSpace /DeviceRGB /BitsPerComponent 8 /Filter /FlateDecode "
               f"/Length {len(raw)} >>").encode()
    img_obj = img_hdr + b"\nstream\n" + raw + b"\nendstream"
    objs.append(img_obj)
    content = (f"q {pw_pt:.3f} 0 0 {ph_pt:.3f} 0 0 cm /Im0 Do Q").encode()
    objs.append((f"<< /Length {len(content)} >>").encode() + b"\nstream\n"
                + content + b"\nendstream")

    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = []
    for i, body in enumerate(objs, 1):
        offsets.append(len(out))
        out += f"{i} 0 obj\n".encode() + body + b"\nendobj\n"
    xref_pos = len(out)
    out += f"xref\n0 {len(objs)+1}\n".encode()
    out += b"0000000000 65535 f \n"
    for off in offsets:
        out += f"{off:010d} 00000 n \n".encode()
    out += (f"trailer\n<< /Size {len(objs)+1} /Root 1 0 R >>\nstartxref\n"
            f"{xref_pos}\n%%EOF\n").encode()

    with open(path, "wb") as f:
        f.write(out)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="charuco_A4.pdf")
    ap.add_argument("--dict", default=BOARD.dictionary)
    ap.add_argument("--squares", default=f"{BOARD.squares_x}x{BOARD.squares_y}",
                    help="columns x rows, e.g. 10x7")
    ap.add_argument("--square-mm", type=float, default=BOARD.square_length_mm)
    ap.add_argument("--marker-mm", type=float, default=BOARD.marker_length_mm)
    ap.add_argument("--dpi", type=int, default=600, help="render DPI (>=600 sharp)")
    args = ap.parse_args()

    page, page_mm = render_page(args)
    path = write_pdf(args.out, page, page_mm)
    sx, sy = (int(v) for v in args.squares.lower().split("x"))
    size_kb = __import__("os").path.getsize(path) / 1024
    print(f"wrote {path}  ({size_kb:.0f} KB)")
    print(f"  board {sx}x{sy}  {args.dict}  square={args.square_mm}mm "
          f"marker={args.marker_mm}mm  ratio={args.marker_mm/args.square_mm:.2f}")
    print(f"  page {page_mm[0]:.0f}x{page_mm[1]:.0f}mm  "
          f"board {sx*args.square_mm:.0f}x{sy*args.square_mm:.0f}mm @ {args.dpi}dpi")
    print("  PRINT AT 100% SCALE, then measure the 50 mm bar to verify.")


if __name__ == "__main__":
    main()
