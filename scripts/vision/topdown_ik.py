#!/usr/bin/env python3
"""Brain-side top-down IK with REACHABLE-branch selection (no robot/serial).

The Pi's solve_topdown_ik returns the first valid least-squares branch, which is
often the elbow-folded branch (J2 near its -135 limit) — physically unreachable:
the arm stalls partway (observed: commands J3=141, stops at 46). For a near-base
top-down point there are typically TWO branches that both put the approach axis
straight down; we must pick the one the hardware can actually hold.

Selection: among branches satisfying downness>=0.98 and pos_err<=tol, prefer the
one with the LARGEST minimum joint-limit margin (stays away from limits), which
empirically is the reachable, non-stalling posture. Returns 6 angles (deg) or None.

Uses the standalone corrected_fk (kinematics.cm_frames); identical DH model to the
Pi, so the joints computed here are valid to send via PUT /joints/angles.
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np
from scipy.optimize import least_squares

import kinematics as K

LIMITS = {1: (-168, 168), 2: (-135, 135), 3: (-150, 150),
          4: (-145, 145), 5: (-155, 160), 6: (-180, 180)}
_LO = np.array([LIMITS[i][0] for i in range(1, 7)], float)
_HI = np.array([LIMITS[i][1] for i in range(1, 7)], float)
_DOWN = np.array([0.0, 0.0, -1.0])

# Seeds spanning both elbow branches so least_squares finds them all.
_SEEDS = [
    [0, 0, -90, 0, 90, 0], [0, -20, -90, 0, 90, 0], [10, -30, -80, 20, 70, 0],
    [0, -60, -30, 0, 90, 0], [-30, -80, -60, 40, 90, 0], [0, -100, 30, 30, 90, 0],
    [0, -120, 60, 0, 90, 0], [-50, -130, 140, -90, 88, 0], [-60, -16, -141, 68, 88, 0],
]


def _resid(q, target):
    fr = K.cm_frames(q)
    ax = fr[5][:3, 2]
    ax = ax / (float(np.linalg.norm(ax)) or 1e-9)
    return list((fr[6][:3, 3] - target) * 0.1) + list((ax - _DOWN) * 40.0)


def _limit_margin(q) -> float:
    """Smallest distance (deg) any joint sits from its nearest limit."""
    return float(min(min(v - LIMITS[i + 1][0], LIMITS[i + 1][1] - v)
                     for i, v in enumerate(q)))


def solve_branches(x, y, z, pos_tol_mm=3.0) -> List[List[float]]:
    """All distinct valid top-down IK branches at (x,y,z), best-margin first."""
    target = np.array([x, y, z], float)
    found = []
    for s in _SEEDS:
        s = np.clip(np.array(s, float), _LO, _HI)
        try:
            sol = least_squares(lambda q: _resid(q, target), s,
                                bounds=(_LO, _HI), method="trf", max_nfev=4000)
        except Exception:
            continue
        q = [float(v) for v in sol.x]
        fr = K.cm_frames(q)
        pos_err = float(np.linalg.norm(fr[6][:3, 3] - target))
        ax = fr[5][:3, 2]
        downness = -float(ax[2]) / (float(np.linalg.norm(ax)) or 1e-9)
        if pos_err > pos_tol_mm or downness < 0.98:
            continue
        if not any(np.allclose(q, u, atol=2.0) for u in found):
            found.append(q)
    found.sort(key=_limit_margin, reverse=True)   # best joint-limit margin first
    return found


def solve_reachable(x, y, z, pos_tol_mm=3.0) -> Optional[List[float]]:
    """Reachable (non-stalling) top-down IK solution, or None."""
    branches = solve_branches(x, y, z, pos_tol_mm)
    return branches[0] if branches else None


if __name__ == "__main__":
    import sys
    x, y, z = (float(v) for v in sys.argv[1:4])
    for q in solve_branches(x, y, z):
        print(f"margin={_limit_margin(q):6.1f}  J={[round(v,1) for v in q]}")
