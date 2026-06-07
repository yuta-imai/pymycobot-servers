#!/usr/bin/env python3
"""Shared configuration for the eye-to-hand SORACAM vision pipeline.

One fixed source of truth for: the ChArUco board geometry (so the printed board,
the calibrators and `locate` all agree), the on-disk calibration artifacts, the
frame source, and the GRASP-HEIGHT CONSTRAINT (we do NOT observe object height
from a single view — it is a configured constant per object class, by design).

Everything here is plain data; importing this module never touches the robot,
the camera, or OpenCV.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))            # project root
ARTIFACT_DIR = os.path.join(HERE, "calib")              # calibration outputs live here

# --- artifact file names (under ARTIFACT_DIR) -------------------------------
INTRINSICS_JSON = "intrinsics.json"      # K, dist, model, image_size  (step 2a)
EXTRINSICS_JSON = "extrinsics.json"      # R_cb, t_cb, board plane in cam (step 2b)
HANDEYE_JSON = "board_to_base.json"      # board->base rigid transform   (step 2c)


# --- ChArUco board (A3) -----------------------------------------------------
# A3 sheet = 297 x 420 mm. We leave a margin and tile an 8 x 11 grid of 35 mm
# squares (8*35=280 mm, 11*35=385 mm) so the printed pattern fits A3 with room
# for a border. The aruco markers sit inside the white squares at 26 mm.
#
# IMPORTANT: print at 100% scale (no "fit to page") and MEASURE one square with
# calipers after printing; put the measured value in SQUARE_LENGTH_MM. A 1.5%
# printer scale error maps almost 1:1 into a metric pick error.
@dataclass(frozen=True)
class BoardSpec:
    squares_x: int = 8                       # columns
    squares_y: int = 11                      # rows
    square_length_mm: float = 35.0           # checker square edge (MEASURE after print)
    marker_length_mm: float = 26.0           # aruco marker edge inside a square
    dictionary: str = "DICT_5X5_1000"        # cv2.aruco predefined dict name
    # The board frame: origin at the board corner, +X along squares_x, +Y along
    # squares_y, +Z out of the board toward the camera. OpenCV's CharucoBoard
    # uses exactly this convention, with chessboard corners at integer multiples
    # of square_length in the XY plane (Z=0).

    @property
    def width_mm(self) -> float:
        return self.squares_x * self.square_length_mm

    @property
    def height_mm(self) -> float:
        return self.squares_y * self.square_length_mm

    @property
    def n_corners(self) -> int:
        # interior chessboard corners
        return (self.squares_x - 1) * (self.squares_y - 1)


BOARD = BoardSpec()


# --- grasp-height constraint -------------------------------------------------
# We do NOT recover object height from one camera. The pick Z is the board plane
# (known in base frame from touch calibration) plus a per-class grasp offset.
# Configure the object class you are picking here. `grasp_z_offset_mm` is how far
# ABOVE the board surface the gripper tip should close (e.g. half the object
# height for a centered top grasp, or the graspable-feature height).
@dataclass(frozen=True)
class GraspProfile:
    name: str = "default-large"
    grasp_z_offset_mm: float = 40.0          # tip closes this far above board plane
    approach_clearance_mm: float = 60.0      # pre-grasp hover above the grasp point
    retreat_mm: float = 80.0                 # lift after grasp


GRASP = GraspProfile()


# --- frame source ------------------------------------------------------------
# Two backends (see frame_source.py):
#   "rtsp"  : cv2.VideoCapture on a stream URL (best for calibration bursts; runs
#             on the Pi / robot host where the camera network is reachable).
#   "mcp"   : decode a base64 still saved from the soracam MCP get_live_still_image
#             tool result (one-shot glances; ~15 s/frame, not for bursts).
@dataclass(frozen=True)
class FrameSourceConfig:
    backend: str = field(default_factory=lambda: os.environ.get("SORACAM_BACKEND", "rtsp"))
    # For "rtsp": full URL. SORACAM unlimited live-view yields a short-lived URL;
    # put it here or in $SORACAM_STREAM_URL. (DASH URLs also work via ffmpeg.)
    stream_url: str = field(default_factory=lambda: os.environ.get("SORACAM_STREAM_URL", ""))
    device_id: str = field(default_factory=lambda: os.environ.get("SORACOM_DEVICE_ID", ""))


FRAME_SOURCE = FrameSourceConfig()


def artifact_path(name: str) -> str:
    return os.path.join(ARTIFACT_DIR, name)


def ensure_artifact_dir() -> str:
    os.makedirs(ARTIFACT_DIR, exist_ok=True)
    return ARTIFACT_DIR
