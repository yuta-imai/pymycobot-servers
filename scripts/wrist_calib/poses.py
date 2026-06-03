#!/usr/bin/env python3
"""Calibration pose generator for the MyCobot 280 wrist.

Goal: enough variety to identify the wrist kinematics AND the constant sensor-mount
rotation, without driving into hard limits or self-collision. All poses branch from
the hardware-verified straight-down "canonical" pose so the arm stays in the same
reachable, gripper-down region of the workspace.

Each pose is a 6-vector of COMMAND joint angles (deg) for send_angles.
"""

from __future__ import annotations

from typing import List

CANONICAL = [0.0, 0.0, -90.0, 0.0, 90.0, 0.0]

# Conservative joint limits (deg) — MyCobot 280, matches the controller.
LIMITS = {1: (-168, 168), 2: (-135, 135), 3: (-150, 150),
          4: (-145, 145), 5: (-155, 160), 6: (-180, 180)}


def _ok(p: List[float]) -> bool:
    return all(LIMITS[i][0] <= a <= LIMITS[i][1] for i, a in enumerate(p, 1))


def _set(base: List[float], idx0: int, val: float) -> List[float]:
    p = list(base)
    p[idx0] = val
    return p


def single_joint_sweeps(step: int = 30) -> List[List[float]]:
    """Sweep J4, J5, J6 one at a time from canonical (the core tilt/roll data)."""
    out = [list(CANONICAL)]
    # J4 (index 3): tilts the approach axis (real: toward +Y)
    for v in range(-120, 121, step):
        out.append(_set(CANONICAL, 3, float(v)))
    # J5 (index 4): canonical is 90; sweep around it (real: tilts toward +X)
    for v in range(0, 161, step):
        out.append(_set(CANONICAL, 4, float(v)))
    # J6 (index 5): pure roll about the approach axis (gravity dir unchanged -> a
    #   strong, clean constraint on the mount rotation + that J6 is a roll)
    for v in range(-150, 151, step):
        out.append(_set(CANONICAL, 5, float(v)))
    return [p for p in out if _ok(p)]


def combined_j4_j5(step: int = 40) -> List[List[float]]:
    """J4 x J5 grid — breaks degeneracies a single-axis sweep can't."""
    out = []
    for a in range(-80, 81, step):
        for b in range(20, 141, step):
            p = list(CANONICAL)
            p[3] = float(a)
            p[4] = float(b)
            if _ok(p):
                out.append(p)
    return out


def base_and_pitch_variants() -> List[List[float]]:
    """Re-orient the whole wrist via J1 (base yaw) and J2/J3 (arm pitch) so the
    calibration generalizes across the workspace, not just at canonical."""
    out = []
    bases = [[j1, j2, j3, 0.0, 90.0, 0.0]
             for j1 in (-40, 0, 40)
             for (j2, j3) in ((0, -90), (-20, -70), (-40, -50), (20, -110))]
    for b in bases:
        if _ok(b):
            out.append(b)
        # add a wrist tilt at each so orientation varies with the base too
        for j4 in (-40, 40):
            p = list(b)
            p[3] = float(j4)
            if _ok(p):
                out.append(p)
    return out


def default_set(step: int = 30) -> List[List[float]]:
    """The full default calibration set (deduplicated)."""
    seen = set()
    out = []
    for p in (single_joint_sweeps(step) + combined_j4_j5() + base_and_pitch_variants()):
        key = tuple(round(x, 1) for x in p)
        if key not in seen:
            seen.add(key)
            out.append(p)
    return out


def quick_set() -> List[List[float]]:
    """A short smoke-test set: canonical + the three sweeps we already did by eye."""
    return [list(CANONICAL),
            _set(CANONICAL, 5, 90.0),    # J6=+90 (roll)
            _set(CANONICAL, 3, -40.0),   # J4=-40
            _set(CANONICAL, 4, 60.0)]    # J5=60


if __name__ == "__main__":
    s = default_set()
    print(f"default_set: {len(s)} poses")
    for p in s:
        print("  ", [round(x) for x in p])
