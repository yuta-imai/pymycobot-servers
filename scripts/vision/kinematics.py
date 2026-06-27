#!/usr/bin/env python3
"""Standalone accelerometer-calibrated forward kinematics (brain-side, no robot).

Mirrors MyCobotJointController._cm_frames/corrected_fk but as a pure
numpy+json module so the BRAIN can turn joint angles (read from the thin Pi API)
into the gripper-tip base pose, without pymycobot or a serial connection. Keeps
"compute on the brain, Pi stays thin" — the Pi only reports raw joint angles.

Loads scripts/wrist_calib/corrected_model.json (committed). Position uses the
official link lengths (absolute position not independently verified); for the
eye-to-hand calibration we only need self-consistency, since board->base is fit
from (board_point, corrected_fk(angles)) pairs and inverted at pick time.
"""

from __future__ import annotations

import json
import math
import os
from typing import List

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODEL_PATH = os.path.join(ROOT, "scripts", "wrist_calib", "corrected_model.json")


def _load(path: str = MODEL_PATH):
    with open(path) as f:
        d = json.load(f)
    return [list(r) for r in d["dh"]], [float(s) for s in d["signs"]]


def _dh(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


_DH, _SIGNS = _load()


def cm_frames(angles_deg: List[float]):
    """DH frames 0..6 (4x4, mm) for the corrected model."""
    T = np.eye(4)
    frames = [T]
    for i in range(6):
        d, a, alpha, off = _DH[i]
        theta = math.radians(angles_deg[i]) * _SIGNS[i] + off
        T = T @ _dh(theta, d, a, alpha)
        frames.append(T)
    return frames


def corrected_fk_pos(angles_deg: List[float]) -> np.ndarray:
    """6 joint angles (deg) -> gripper-tip position [x,y,z] mm in base frame."""
    return cm_frames(angles_deg)[6][:3, 3].copy()


def downness(angles_deg: List[float]) -> float:
    """How vertical the gripper approach axis is: +1 = straight down."""
    z = cm_frames(angles_deg)[5][:3, 2]
    z = z / (float(np.linalg.norm(z)) or 1e-9)
    return -float(z[2])


if __name__ == "__main__":
    # sanity: FK of all-zeros and a sample pose
    for a in ([0, 0, 0, 0, 0, 0], [-60, -30, -120, 90, 80, 0]):
        print(a, "-> tip", corrected_fk_pos(a).round(1), "downness", round(downness(a), 3))
