#!/usr/bin/env python3
"""Pure-geometry core for the eye-to-hand pipeline (numpy only, no cv2/robot).

All the math that turns a board-plane PnP pose + a board->base rigid transform +
an (undistorted) pixel ray into a base-frame grasp point lives here, plus the
Kabsch/Umeyama rigid fit used by touch calibration. Kept cv2-free so it is unit
testable on a box without OpenCV (run `python geometry.py` for self-tests).

Conventions (match OpenCV + the project's base frame, all lengths in mm):
- A homogeneous transform T (4x4) maps a point p in frame A to frame B:
  p_B = T @ [p_A; 1].  T_B_A reads "A expressed in B".
- Camera frame: +Z forward (into scene), pinhole at origin, OpenCV image axes.
- Board frame: origin at a board corner, board surface is the z=0 plane,
  +Z out of the board toward the camera.
- Base frame: the robot base (pymycobot coords convention), +Z world-up.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np


# --------------------------------------------------------------------------- #
# Homogeneous transforms
# --------------------------------------------------------------------------- #
def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Build a 4x4 from rotation R (3x3) and translation t (3,)."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid 4x4 (transpose rotation, rotate-negate translation)."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply 4x4 T to an (N,3) array (or a single (3,) point)."""
    pts = np.asarray(pts, float)
    single = pts.ndim == 1
    P = np.atleast_2d(pts)
    out = (T[:3, :3] @ P.T).T + T[:3, 3]
    return out[0] if single else out


# --------------------------------------------------------------------------- #
# Kabsch / Umeyama rigid fit (no scale) — board<->base from touch points
# --------------------------------------------------------------------------- #
def kabsch(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Best-fit rigid transform T (4x4) s.t. T @ src ~= dst (least squares).

    src, dst: (N,3) corresponding points (N >= 3, non-collinear). Returns the
    transform mapping the src frame into the dst frame.
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("need matching (N>=3, 3) point sets")
    cs = src.mean(0)
    cd = dst.mean(0)
    S = src - cs
    D = dst - cd
    H = S.T @ D
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    Dg = np.diag([1.0, 1.0, d])
    R = Vt.T @ Dg @ U.T
    t = cd - R @ cs
    return make_transform(R, t)


def rigid_fit_residuals(T: np.ndarray, src: np.ndarray, dst: np.ndarray
                        ) -> Tuple[float, float, np.ndarray]:
    """Return (rms_mm, max_mm, per_point_mm) for T @ src vs dst."""
    pred = transform_points(T, src)
    err = np.linalg.norm(pred - np.asarray(dst, float), axis=1)
    return float(np.sqrt(np.mean(err ** 2))), float(err.max()), err


# --------------------------------------------------------------------------- #
# Ray / plane
# --------------------------------------------------------------------------- #
def normalized_pixel_to_ray(xn: float, yn: float) -> np.ndarray:
    """Undistorted normalized image coords (x',y') -> unit ray dir in cam frame.

    Normalized coords are what cv2.undistortPoints(...,P=None) returns: the pixel
    after removing K and distortion, i.e. (X/Z, Y/Z) of the camera ray.
    """
    d = np.array([xn, yn, 1.0], float)
    return d / np.linalg.norm(d)


def ray_plane_intersection(origin: np.ndarray, direction: np.ndarray,
                           plane_point: np.ndarray, plane_normal: np.ndarray
                           ) -> np.ndarray:
    """Intersect ray origin+s*dir (s>=0) with a plane. Raises if near-parallel."""
    o = np.asarray(origin, float)
    d = np.asarray(direction, float)
    p0 = np.asarray(plane_point, float)
    n = np.asarray(plane_normal, float)
    denom = float(n @ d)
    if abs(denom) < 1e-9:
        raise ValueError("ray parallel to plane")
    s = float(n @ (p0 - o)) / denom
    if s <= 0:
        raise ValueError(f"plane behind camera (s={s:.3f})")
    return o + s * d


# --------------------------------------------------------------------------- #
# Full pixel -> base-frame point, given the three calibrations
# --------------------------------------------------------------------------- #
def board_point_from_ray(ray_dir_cam: np.ndarray, T_cam_board: np.ndarray
                         ) -> np.ndarray:
    """Intersect a camera ray (from the pinhole) with the board's z=0 plane.

    T_cam_board: board expressed in camera frame (PnP rvec/tvec composed).
    Returns the hit point in BOARD coordinates (z ~ 0).
    """
    plane_point = T_cam_board[:3, 3]            # board origin in cam
    plane_normal = T_cam_board[:3, 2]           # board +Z in cam
    P_cam = ray_plane_intersection(np.zeros(3), ray_dir_cam,
                                   plane_point, plane_normal)
    return transform_points(invert_transform(T_cam_board), P_cam)


def base_grasp_point(ray_dir_cam: np.ndarray, T_cam_board: np.ndarray,
                     T_base_board: np.ndarray, grasp_z_offset_mm: float
                     ) -> Tuple[np.ndarray, np.ndarray]:
    """Pixel ray -> (footprint_base_xyz, grasp_base_xyz).

    footprint is where the ray meets the board surface, in base mm. grasp lifts
    that point by grasp_z_offset_mm along base +Z (world up) for a top-down grasp.
    """
    P_board = board_point_from_ray(ray_dir_cam, T_cam_board)
    P_board[2] = 0.0                            # snap to board plane
    foot = transform_points(T_base_board, P_board)
    grasp = foot.copy()
    grasp[2] += grasp_z_offset_mm
    return foot, grasp


# --------------------------------------------------------------------------- #
# Self-tests (run directly): synthetic round-trips with known ground truth.
# --------------------------------------------------------------------------- #
def _rot_xyz(rx, ry, rz):
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _selftest():
    rng = np.random.default_rng(0)
    # 1) Kabsch recovers a known rigid transform from noisy points.
    R_true = _rot_xyz(0.2, -0.5, 1.1)
    t_true = np.array([120.0, -45.0, 300.0])
    T_true = make_transform(R_true, t_true)
    src = rng.uniform(-150, 150, size=(8, 3))
    dst = transform_points(T_true, src) + rng.normal(0, 0.3, size=(8, 3))
    T_fit = kabsch(src, dst)
    rms, mx, _ = rigid_fit_residuals(T_fit, src, dst)
    assert rms < 1.0, rms
    assert np.allclose(T_fit, T_true, atol=2.0), "transform off"
    print(f"[kabsch] rms={rms:.3f}mm max={mx:.3f}mm  (0.3mm noise) OK")

    # 2) ray-plane: a pixel ray hits a known board point, recovered exactly.
    #    Build a board pose in camera frame, pick a board point, project its ray.
    T_cam_board = make_transform(_rot_xyz(0.1, 0.2, -0.3),
                                 np.array([30.0, -20.0, 800.0]))
    P_board_true = np.array([140.0, 200.0, 0.0])          # on board plane
    P_cam = transform_points(T_cam_board, P_board_true)
    ray = P_cam / np.linalg.norm(P_cam)                    # ray from pinhole
    P_board_rec = board_point_from_ray(ray, T_cam_board)
    assert np.allclose(P_board_rec, P_board_true, atol=1e-6), P_board_rec
    print(f"[ray-plane] recovered board point {P_board_rec.round(4)} OK")

    # 3) full chain pixel->base with a known board->base.
    T_base_board = make_transform(_rot_xyz(0.0, 0.0, 0.7),
                                  np.array([200.0, 0.0, -50.0]))
    foot, grasp = base_grasp_point(ray, T_cam_board, T_base_board, 40.0)
    foot_true = transform_points(T_base_board, P_board_true)
    assert np.allclose(foot, foot_true, atol=1e-6)
    assert abs((grasp[2] - foot[2]) - 40.0) < 1e-9
    print(f"[chain] footprint_base={foot.round(2)} grasp_z=+40 OK")
    print("all geometry self-tests passed")


if __name__ == "__main__":
    _selftest()
