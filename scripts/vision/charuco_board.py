#!/usr/bin/env python3
"""ChArUco board definition + printable generator (shared truth for the board).

The SAME board object is used to (a) generate the PNG you print and (b) detect
corners in calibration/locate, so the geometry can never disagree. Board params
come from config.BOARD. Run directly to write a print-ready PNG:

    python charuco_board.py --out charuco_A3.png --dpi 300

Then print at 100% scale on A3, MEASURE one square with calipers, and put the
measured edge length into config.BOARD.square_length_mm before calibrating.

Requires OpenCV (cv2) with the aruco module — present on the robot host / Pi
(`pip install opencv-python`), not necessarily on a dev box.
"""

from __future__ import annotations

import argparse
from typing import Tuple

import numpy as np

from config import BOARD


def _aruco():
    import cv2
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV built without the aruco module")
    return cv2.aruco


def get_dictionary():
    aruco = _aruco()
    return aruco.getPredefinedDictionary(getattr(aruco, BOARD.dictionary))


def build_board():
    """Return (dictionary, CharucoBoard) using config.BOARD, mm as the length unit.

    Lengths are passed in millimetres so getChessboardCorners() returns board-
    frame corner coordinates directly in mm (z=0 plane).
    """
    aruco = _aruco()
    dictionary = get_dictionary()
    size = (BOARD.squares_x, BOARD.squares_y)
    try:                                   # OpenCV >= 4.7
        board = aruco.CharucoBoard(size, BOARD.square_length_mm,
                                   BOARD.marker_length_mm, dictionary)
    except AttributeError:                 # OpenCV < 4.7 fallback
        board = aruco.CharucoBoard_create(BOARD.squares_x, BOARD.squares_y,
                                          BOARD.square_length_mm,
                                          BOARD.marker_length_mm, dictionary)
    return dictionary, board


def chessboard_corners_mm(board) -> np.ndarray:
    """(N,3) interior chessboard corner coordinates in board frame (mm, z=0).

    Index == ChArUco corner id, which is how the detector reports them, so this
    array is the lookup from detected id -> board-frame point.
    """
    try:
        corners = board.getChessboardCorners()      # OpenCV >= 4.7
    except AttributeError:
        corners = board.chessboardCorners           # older
    return np.asarray(corners, dtype=float).reshape(-1, 3)


def make_detector(board):
    """Return a callable detect(gray) -> (charuco_corners_px, charuco_ids) using
    whichever API the installed OpenCV exposes. Returns (None, None) on no/low
    detection."""
    import cv2
    aruco = _aruco()
    dictionary = get_dictionary()

    if hasattr(aruco, "CharucoDetector"):           # OpenCV >= 4.7
        detector = aruco.CharucoDetector(board)

        def detect(gray):
            ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
            if ch_ids is None or len(ch_ids) < 4:
                return None, None
            return ch_corners, ch_ids
        return detect

    # Legacy API
    params = aruco.DetectorParameters_create()

    def detect(gray):
        m_corners, m_ids, _ = aruco.detectMarkers(gray, dictionary,
                                                  parameters=params)
        if m_ids is None or len(m_ids) == 0:
            return None, None
        n, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
            m_corners, m_ids, gray, board)
        if ch_ids is None or n < 4:
            return None, None
        return ch_corners, ch_ids
    return detect


def board_px_size(dpi: int) -> Tuple[int, int]:
    """Pixel size of the bare board grid at a given DPI (no margin)."""
    mm_to_px = dpi / 25.4
    return (int(round(BOARD.width_mm * mm_to_px)),
            int(round(BOARD.height_mm * mm_to_px)))


def generate_png(out_path: str, dpi: int = 300, margin_mm: float = 10.0) -> str:
    import cv2
    _, board = build_board()
    w_px, h_px = board_px_size(dpi)
    margin_px = int(round(margin_mm * dpi / 25.4))
    try:                                   # OpenCV >= 4.7
        img = board.generateImage((w_px, h_px), marginSize=margin_px)
    except AttributeError:                 # older signature
        img = board.draw((w_px, h_px), marginSize=margin_px)
    cv2.imwrite(out_path, img)
    return out_path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="charuco_A3.png")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--margin-mm", type=float, default=10.0)
    args = ap.parse_args()
    path = generate_png(args.out, args.dpi, args.margin_mm)
    print(f"wrote {path}")
    print(f"board grid = {BOARD.width_mm:.0f} x {BOARD.height_mm:.0f} mm "
          f"({BOARD.squares_x}x{BOARD.squares_y} squares @ "
          f"{BOARD.square_length_mm} mm), {BOARD.n_corners} interior corners")
    print("PRINT AT 100% SCALE on A3, then measure one square and update "
          "config.BOARD.square_length_mm.")


if __name__ == "__main__":
    main()
