#!/usr/bin/env python3
"""Fit a corrected MyCobot 280 kinematic model from accelerometer calibration data.

Model per pose, arm static:
    g_sensor = R_mount^T . R_flange(q; params)^T . g_world
with g_world = R_basetilt . [0,0,-1]. We fit:
  - DH corrections (d, a, alpha, theta_offset deltas) on a chosen joint set,
  - R_mount (3-param rotvec): the constant, unknown sensor->flange rotation,
  - R_basetilt (2-param): base not perfectly level (optional),
so that predicted gravity matches the measured gravity across ALL poses, while the
tip POSITION still matches get_coords (position residual keeps the good position FK
from drifting). k-fold cross-validation guards against overfitting.

Offline only (numpy + scipy, already in the venv). No robot, no BLE.

Usage:
  python3 scripts/wrist_calib/calibrate.py --data calib.jsonl --fit-joints 4 5 6
  python3 scripts/wrist_calib/calibrate.py --selftest      # synthetic sanity check
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import List, Tuple

import numpy as np

DEG = math.pi / 180.0

# Official standard-DH table (GitBook), d(mm), a(mm), alpha(rad), theta_offset(rad).
DH0 = [
    (131.22, 0.0, math.pi / 2, 0.0),
    (0.0, -110.4, 0.0, -math.pi / 2),
    (0.0, -96.0, 0.0, 0.0),
    (63.4, 0.0, math.pi / 2, -math.pi / 2),
    (75.05, 0.0, -math.pi / 2, math.pi / 2),
    (45.6, 0.0, 0.0, 0.0),
]


def dh_matrix(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def rotvec_to_R(rv):
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(rv) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


class WristModel:
    """DH model with fittable corrections + sensor mount + base tilt."""

    def __init__(self, fit_joints=(4, 5, 6), fit_params=("alpha", "theta"),
                 fit_basetilt=False):
        self.fit_joints = tuple(fit_joints)      # 1-based joint indices to correct
        self.fit_params = tuple(fit_params)      # subset of d,a,alpha,theta
        self.fit_basetilt = fit_basetilt
        self._pk = [p for p in ("d", "a", "alpha", "theta") if p in fit_params]

    # ---- parameter vector packing --------------------------------------
    def n_params(self):
        return len(self.fit_joints) * len(self._pk) + 3 + (2 if self.fit_basetilt else 0)

    def x0(self):
        return np.zeros(self.n_params())

    def _unpack(self, x):
        DH = [list(r) for r in DH0]
        idx = 0
        order = {"d": 0, "a": 1, "alpha": 2, "theta": 3}
        for j in self.fit_joints:
            for pk in self._pk:
                DH[j - 1][order[pk]] += x[idx]
                idx += 1
        rmount = rotvec_to_R(x[idx:idx + 3]); idx += 3
        if self.fit_basetilt:
            tx, ty = x[idx], x[idx + 1]; idx += 2
            Rbt = rotvec_to_R([tx, ty, 0.0])
        else:
            Rbt = np.eye(3)
        return DH, rmount, Rbt

    # ---- kinematics -----------------------------------------------------
    @staticmethod
    def _fk(DH, q_deg):
        T = np.eye(4)
        for (d, a, al, off), q in zip(DH, q_deg):
            T = T @ dh_matrix(q * DEG + off, d, a, al)
        return T

    def flange(self, x, q_deg):
        DH, _, _ = self._unpack(x)
        return self._fk(DH, q_deg)

    def predict_gravity(self, x, q_deg):
        DH, rmount, Rbt = self._unpack(x)
        Rfl = self._fk(DH, q_deg)[:3, :3]
        gw = Rbt @ np.array([0.0, 0.0, -1.0])
        g = rmount.T @ Rfl.T @ gw
        return g / (np.linalg.norm(g) or 1e-9)

    def approach_axis(self, x, q_deg):
        """J6 rotation axis (= the gripper approach axis) in world frame."""
        DH, _, _ = self._unpack(x)
        z5 = self._fk(DH, q_deg)  # frame 6; J6 axis is z of frame 5
        # recompute frame-5 z explicitly:
        T = np.eye(4)
        for k in range(5):
            d, a, al, off = DH[k]
            T = T @ dh_matrix(q_deg[k] * DEG + off, d, a, al)
        z = T[:3, 2]
        return z / (np.linalg.norm(z) or 1e-9)


def load_data(path) -> List[dict]:
    recs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                recs.append(json.loads(line))
    return recs


def residuals(x, model: WristModel, recs, w_pos=0.05, w_reg=0.5):
    r = []
    for rec in recs:
        q = rec.get("angles") or rec["commanded"]
        g_meas = np.array(rec["gravity"], dtype=float)
        g_meas = g_meas / (np.linalg.norm(g_meas) or 1e-9)
        g_pred = model.predict_gravity(x, q)
        # sign-agnostic per pose: align to whichever sign is closer (mount absorbs it)
        if float(g_pred @ g_meas) < 0:
            g_pred = -g_pred
        r += list(g_pred - g_meas)                       # unit-vector residual (~rad)
        coords = rec.get("coords") or []
        if len(coords) >= 3 and all(c is not None for c in coords[:3]):
            pos = model.flange(x, q)[:3, 3]
            r += list((pos - np.array(coords[:3], float)) * w_pos)  # mm * w
    r += list(np.asarray(x) * w_reg)                     # regularize toward official
    return r


def fit(model: WristModel, recs, **kw):
    from scipy.optimize import least_squares
    sol = least_squares(residuals, model.x0(), args=(model, recs),
                        kwargs=kw, method="lm", max_nfev=40000)
    return sol


def report(model, x, recs, tag=""):
    ang_err, pos_err = [], []
    for rec in recs:
        q = rec.get("angles") or rec["commanded"]
        gm = np.array(rec["gravity"], float); gm /= (np.linalg.norm(gm) or 1e-9)
        gp = model.predict_gravity(x, q)
        if float(gp @ gm) < 0:
            gp = -gp
        ang_err.append(math.degrees(math.acos(max(-1, min(1, float(gp @ gm))))))
        coords = rec.get("coords") or []
        if len(coords) >= 3 and all(c is not None for c in coords[:3]):
            pos_err.append(float(np.linalg.norm(model.flange(x, q)[:3, 3]
                                                - np.array(coords[:3], float))))
    a = np.array(ang_err); p = np.array(pos_err) if pos_err else np.array([0.0])
    print(f"  {tag:10s} orient err deg: mean {a.mean():5.2f} max {a.max():5.2f}"
          f"   pos err mm: mean {p.mean():5.2f} max {p.max():5.2f}  (n={len(recs)})")
    return a.mean(), a.max()


def kfold(model: WristModel, recs, k=5, **kw):
    n = len(recs)
    idx = list(range(n))
    folds = [idx[i::k] for i in range(k)]
    test_errs = []
    for f in range(k):
        test = [recs[i] for i in folds[f]]
        train = [recs[i] for i in idx if i not in set(folds[f])]
        if not train or not test:
            continue
        sol = fit(model, train, **kw)
        m, mx = report(model, sol.x, test, tag=f"fold{f} test")
        test_errs.append(m)
    if test_errs:
        print(f"  CV mean test orient err: {np.mean(test_errs):.2f} deg")


# --------------------------------------------------------------------------
def selftest():
    """Generate synthetic data from a KNOWN perturbed model + mount, then recover."""
    rng_axis = [0.10, -0.20, 0.05]  # pretend wrist offsets we will recover (rad-ish)
    truth = WristModel(fit_joints=(4, 5, 6), fit_params=("alpha", "theta"))
    xt = np.zeros(truth.n_params())
    xt[0], xt[1] = 0.15, -0.10     # J4 dAlpha, dTheta
    xt[2], xt[3] = -0.12, 0.20     # J5
    xt[4], xt[5] = 0.05, 0.08      # J6
    xt[6:9] = [0.3, -0.2, 0.1]     # mount rotvec
    import itertools
    recs = []
    for j4 in range(-90, 91, 30):
        for j5 in range(30, 151, 30):
            for j6 in range(-90, 91, 90):
                q = [0, 0, -90, float(j4), float(j5), float(j6)]
                g = truth.predict_gravity(xt, q)
                pos = truth.flange(xt, q)[:3, 3]
                recs.append({"angles": q, "gravity": [float(v) for v in g],
                             "coords": [float(v) for v in pos] + [0, 0, 0]})
    print(f"[selftest] {len(recs)} synthetic poses")
    model = WristModel(fit_joints=(4, 5, 6), fit_params=("alpha", "theta"))
    sol = fit(model, recs)
    print("[selftest] recovered params (should match truth):")
    print("  truth:", [round(v, 3) for v in xt])
    print("  fit  :", [round(v, 3) for v in sol.x])
    report(model, model.x0(), recs, tag="before")
    report(model, sol.x, recs, tag="after")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data")
    ap.add_argument("--fit-joints", type=int, nargs="+", default=[4, 5, 6])
    ap.add_argument("--fit-params", nargs="+", default=["alpha", "theta"],
                    choices=["d", "a", "alpha", "theta"])
    ap.add_argument("--basetilt", action="store_true")
    ap.add_argument("--cv", type=int, default=5)
    ap.add_argument("--out", help="write fitted params JSON")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        selftest(); return 0
    if not args.data:
        ap.error("--data required (or use --selftest)")

    recs = load_data(args.data)
    recs = [r for r in recs if r.get("still", True)]
    print(f"[calibrate] {len(recs)} usable poses from {args.data}")
    model = WristModel(tuple(args.fit_joints), tuple(args.fit_params), args.basetilt)
    report(model, model.x0(), recs, tag="before")
    sol = fit(model, recs)
    report(model, sol.x, recs, tag="after")
    if args.cv and len(recs) >= args.cv:
        print("[calibrate] cross-validation:")
        kfold(model, recs, k=args.cv)
    if args.out:
        with open(args.out, "w") as f:
            json.dump({"fit_joints": args.fit_joints, "fit_params": args.fit_params,
                       "basetilt": args.basetilt, "params": list(sol.x),
                       "dh0": DH0}, f, indent=2)
        print(f"[calibrate] wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
