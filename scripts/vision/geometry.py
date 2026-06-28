#!/usr/bin/env python3
"""Pure-geometry core for the homography eye-to-hand pipeline (numpy only).

The fixed camera + planar work surface (objects on the board) let us skip lens
intrinsics entirely: a single planar homography maps image pixels <-> board-plane
mm, absorbing distortion within the board region. This module holds the math that
is cv2-free and unit-testable (run `python geometry.py`):

  - apply_homography: pixel <-> board-plane point via a 3x3 H
  - umeyama / kabsch: board->base fit from touch points (optional scale)
  - transforms: compose/invert/apply 4x4 rigid(+scale) transforms

Conventions (mm):
- A 4x4 transform T maps p_A -> p_B: p_B = T @ [p_A;1]. T_B_A = "A in B".
- Board frame: board surface is z=0; coords in mm (set by square_length_mm).
- Base frame: robot base (pymycobot coords), +Z world-up.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np


# --------------------------------------------------------------------------- #
# 4x4 transforms (rigid, or similarity when a scale is baked into the 3x3 block)
# --------------------------------------------------------------------------- #
def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid 4x4 (no scale). Use for pure rotation+translation."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply 4x4 T to (N,3) or a single (3,). Works for similarity too."""
    pts = np.asarray(pts, float)
    single = pts.ndim == 1
    P = np.atleast_2d(pts)
    out = (T[:3, :3] @ P.T).T + T[:3, 3]
    return out[0] if single else out


# --------------------------------------------------------------------------- #
# Planar homography: image pixel <-> board-plane (mm)
# --------------------------------------------------------------------------- #
def apply_homography(H: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a 3x3 homography to (N,2) points (or one (2,)), with perspective
    divide. H maps the input plane to the output plane."""
    pts = np.asarray(pts, float)
    single = pts.ndim == 1
    P = np.atleast_2d(pts)
    hom = np.hstack([P, np.ones((len(P), 1))])
    out = (H @ hom.T).T
    out = out[:, :2] / out[:, 2:3]
    return out[0] if single else out


# --------------------------------------------------------------------------- #
# Umeyama / Kabsch rigid(+scale) fit — board<->base from touch points
# --------------------------------------------------------------------------- #
def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool = False
           ) -> Tuple[np.ndarray, float]:
    """Best similarity (with_scale) or rigid transform T s.t. T@src ~= dst.

    Returns (T 4x4, scale). With with_scale=True the 3x3 block is scale*R, which
    can absorb a wrong board-mm scale (e.g. an unmeasured square size). src,dst:
    (N,3), N>=3 non-collinear.
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("need matching (N>=3, 3) point sets")
    n = src.shape[0]
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    H = (S.T @ D) / n
    U, sv, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    Dg = np.diag([1.0, 1.0, d])
    R = Vt.T @ Dg @ U.T
    if with_scale:
        var_s = (S ** 2).sum() / n
        scale = float((sv * np.array([1.0, 1.0, d])).sum() / var_s)
    else:
        scale = 1.0
    t = mu_d - scale * R @ mu_s
    return make_transform(scale * R, t), scale


def kabsch(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Rigid (scale=1) fit T s.t. T@src ~= dst. Thin wrapper over umeyama."""
    return umeyama(src, dst, with_scale=False)[0]


def rigid_fit_residuals(T: np.ndarray, src: np.ndarray, dst: np.ndarray
                        ) -> Tuple[float, float, np.ndarray]:
    """(rms_mm, max_mm, per_point_mm) for T @ src vs dst."""
    pred = transform_points(T, src)
    err = np.linalg.norm(pred - np.asarray(dst, float), axis=1)
    return float(np.sqrt(np.mean(err ** 2))), float(err.max()), err


# --------------------------------------------------------------------------- #
# board -> base as homography (x,y) + plane (z)
# --------------------------------------------------------------------------- #
# The arm's corrected_fk position is not metric-accurate (this unit's absolute
# position is uncalibrated), so a RIGID board->base does not fit (RMS ~20mm on
# hardware). Empirically the corrected_fk-space is ~projective over the board, so
# board(X,Y) -> base(x,y) is fit as a 2D HOMOGRAPHY and base_z as a tilted PLANE.
# Because solve_topdown_ik targets that SAME corrected_fk space, feeding it these
# (x,y,z) drives the PHYSICAL tip onto the board point despite the FK distortion.
def fit_plane(xy: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Least-squares plane z = a*X + b*Y + c. Returns (a,b,c)."""
    xy = np.asarray(xy, float)
    A = np.column_stack([xy, np.ones(len(xy))])
    coef, *_ = np.linalg.lstsq(A, np.asarray(z, float), rcond=None)
    return coef


def apply_plane(plane: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Evaluate z = a*X + b*Y + c at (N,2) xy (or one (2,))."""
    xy = np.asarray(xy, float)
    single = xy.ndim == 1
    P = np.atleast_2d(xy)
    z = P @ np.asarray(plane[:2], float) + plane[2]
    return float(z[0]) if single else z


# --------------------------------------------------------------------------- #
# Full chain: board-plane pixel -> base-frame grasp point
# --------------------------------------------------------------------------- #
def board_to_base_fn(board_to_base):
    """Return a callable f(board_xy) -> base_xy from a board->base model.

    Accepts (so callers stay agnostic to the calibration kind):
      - a callable: returned as-is
      - a 3x3 ndarray: a plain homography
      - a dict (loaded board_to_base.json): model="homography+plane" uses
        H_board2base; model starting with "rbf" rebuilds an RBFInterpolator from
        the stored observed_board_mm/commanded_base_mm. The board->base map is
        NON-projective (corrected_fk position distortion), so a thin-plate spline
        fits far better than a homography — see calib/board_to_base_rbf.json.
    f accepts (2,) or (N,2) and returns the same shape.
    """
    if callable(board_to_base):
        return board_to_base
    if isinstance(board_to_base, np.ndarray):
        H = board_to_base
        return lambda pts: apply_homography(H, pts)
    d = board_to_base
    if str(d.get("model", "homography+plane")).startswith("rbf"):
        from scipy.interpolate import RBFInterpolator
        O = np.asarray(d["observed_board_mm"], float)
        B = np.asarray(d["commanded_base_mm"], float)
        rbf = RBFInterpolator(O, B, kernel=d.get("kernel", "thin_plate_spline"),
                              smoothing=float(d.get("smoothing", 1.0)))

        def _f(pts):
            pts = np.asarray(pts, float)
            single = pts.ndim == 1
            out = rbf(np.atleast_2d(pts))
            return out[0] if single else out
        return _f
    H = np.asarray(d["H_board2base"], float)
    return lambda pts: apply_homography(H, pts)


def base_grasp_point(pixel: np.ndarray, H_img2board: np.ndarray,
                     H_board2base, z_plane: np.ndarray,
                     grasp_z_offset_mm: float) -> Tuple[np.ndarray, np.ndarray]:
    """Object board-contact pixel -> (footprint_base_xyz, grasp_base_xyz).

    pixel -> board (X,Y) via H_img2board -> base (x,y) via the board->base model
    (homography OR RBF; see board_to_base_fn), base z via the fitted plane. grasp
    lifts the footprint by grasp_z_offset_mm in corrected_fk-space z (which
    solve_topdown_ik targets).
    """
    bxy = apply_homography(H_img2board, np.asarray(pixel, float).reshape(1, 2))[0]
    base_xy = board_to_base_fn(H_board2base)(bxy.reshape(1, 2))[0]
    base_z = apply_plane(z_plane, bxy)
    foot = np.array([base_xy[0], base_xy[1], base_z])
    grasp = np.array([base_xy[0], base_xy[1], base_z + grasp_z_offset_mm])
    return foot, grasp


# --------------------------------------------------------------------------- #
# Self-tests
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
    T_true = make_transform(_rot_xyz(0.2, -0.5, 1.1), np.array([120., -45., 300.]))
    src = rng.uniform(-150, 150, size=(8, 3))
    dst = transform_points(T_true, src) + rng.normal(0, 0.3, size=(8, 3))
    T_fit = kabsch(src, dst)
    rms, mx, _ = rigid_fit_residuals(T_fit, src, dst)
    assert rms < 1.0 and np.allclose(T_fit, T_true, atol=2.0)
    print(f"[kabsch] rms={rms:.3f}mm (0.3mm noise) OK")

    # 2) Umeyama recovers a known scale (e.g. wrong board-mm scale).
    s_true = 1.07
    T_s = make_transform(s_true * _rot_xyz(0, 0, 0.6), np.array([10., 20., -5.]))
    dst_s = transform_points(T_s, src)
    T_rec, s_rec = umeyama(src, dst_s, with_scale=True)
    assert abs(s_rec - s_true) < 1e-6 and np.allclose(T_rec, T_s, atol=1e-6)
    print(f"[umeyama] recovered scale {s_rec:.4f} (true {s_true}) OK")

    # 3) Homography round-trip: board<->image, recover board pts exactly.
    H_b2i = np.array([[2.0, 0.1, 300.], [0.05, 1.9, 200.], [1e-4, 2e-4, 1.0]])
    board_pts = rng.uniform(0, 280, size=(12, 2))
    px = apply_homography(H_b2i, board_pts)
    H_i2b = np.linalg.inv(H_b2i)
    rec = apply_homography(H_i2b, px)
    assert np.allclose(rec, board_pts, atol=1e-8)
    print("[homography] image<->board round-trip OK")

    # 4) plane fit/eval round-trip
    xy = rng.uniform(0, 250, size=(8, 2))
    plane_true = np.array([0.05, -0.03, 90.0])
    z = apply_plane(plane_true, xy)
    assert np.allclose(fit_plane(xy, z), plane_true, atol=1e-9)
    print("[plane] fit/eval round-trip OK")

    # 5) Full chain: pixel -> board -> base(homography xy) + plane z.
    H_board2base = np.array([[0.5, 0.02, 60.], [-0.03, 0.55, -120.],
                             [2e-5, 1e-5, 1.0]])
    z_plane = np.array([0.04, -0.02, 95.0])
    pix = apply_homography(H_b2i, np.array([[140., 100.]]))[0]
    foot, grasp = base_grasp_point(pix, H_i2b, H_board2base, z_plane, 40.0)
    bxy_true = np.array([140., 100.])
    base_xy_true = apply_homography(H_board2base, bxy_true.reshape(1, 2))[0]
    assert np.allclose(foot[:2], base_xy_true, atol=1e-6)
    assert abs(foot[2] - apply_plane(z_plane, bxy_true)) < 1e-9
    assert abs((grasp[2] - foot[2]) - 40.0) < 1e-9
    print(f"[chain] footprint_base={foot.round(2)} grasp_z=+40 OK")
    print("all geometry self-tests passed")


if __name__ == "__main__":
    _selftest()
