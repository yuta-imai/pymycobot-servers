#!/usr/bin/env python3
"""Build a MyCobot 280 kinematic chain from the OFFICIAL DH table and verify it
against hardware-confirmed facts.

WHY: the vendored URDF wrist origins are inconsistent with this unit (a J4 sweep
tilts the gripper in the wrong plane; patching one origin breaks J6 downstream).
The official Elephant Robotics GitBook DH table is internally consistent, so we
rebuild the chain from it instead.

Official standard-DH table (GitBook Kinematics&Coordinate), command = q_i + offset:
  joint  d(mm)   a(mm)    alpha(rad)  offset(rad)
   1     131.22    0       +pi/2        0
   2       0     -110.4     0          -pi/2
   3       0      -96       0           0
   4      63.4     0       +pi/2       -pi/2
   5      75.05    0       -pi/2       +pi/2
   6      45.6     0        0           0

Hardware-confirmed anchors to check against (no URDF needed):
  - gripper approach axis = the tool axis that points world-down at the known pose
  - straight-down pose = joints [0,0,-90,0,90,0]
  - J6 spins the gripper about its own axis (approach axis unchanged)
  - J5 from that pose tilts approach ~30 deg toward base +X (camera/forward)
  - J4 from that pose tilts approach ~40 deg toward base +Y (camera-left)

This script is OFFLINE (no robot, no serial) — pure model check. It prints the
approach-axis behaviour for the anchor poses so we can confirm the DH chain
matches the real arm before wiring it into the controller.

Usage:
  python3 scripts/verify_dh.py
"""

import math
import numpy as np

DEG = math.pi / 180.0

# d(mm), a(mm), alpha(rad), theta_offset(rad)
DH = [
    (131.22,    0.0,  math.pi / 2,  0.0),
    (0.0,    -110.4,  0.0,         -math.pi / 2),
    (0.0,     -96.0,  0.0,          0.0),
    (63.4,      0.0,  math.pi / 2, -math.pi / 2),
    (75.05,     0.0, -math.pi / 2,  math.pi / 2),
    (45.6,      0.0,  0.0,          0.0),
]


def dh_matrix(theta, d, a, alpha):
    """Standard (classic) DH homogeneous transform, lengths in mm."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,      ca,      d],
        [0.0,     0.0,     0.0,    1.0],
    ])


def fk(joint_deg):
    """Forward kinematics from 6 command angles (deg). Returns 4x4 (mm)."""
    T = np.eye(4)
    for (d, a, alpha, off), q in zip(DH, joint_deg):
        T = T @ dh_matrix(q * DEG + off, d, a, alpha)
    return T


def axis_label(v):
    k = max(range(3), key=lambda i: abs(v[i]))
    names = {(0, 1): "+X(fwd)", (0, -1): "-X(back)", (1, 1): "+Y(left)",
             (1, -1): "-Y(right)", (2, 1): "+Z(up)", (2, -1): "-Z(DOWN)"}
    return names[(k, 1 if v[k] >= 0 else -1)]


def downness(col):
    n = np.linalg.norm(col) or 1e-9
    return -col[2] / n


def tilt_deg(col):
    return math.degrees(math.acos(max(-1.0, min(1.0, downness(col)))))


def main():
    print("=== MyCobot 280 DH chain — FK at anchor poses ===\n")

    down = [0, 0, -90, 0, 90, 0]
    T = fk(down)
    R = T[:3, :3]
    print(f"straight-down pose {down}:")
    print(f"  tip pos (mm): {[round(v, 1) for v in T[:3, 3]]}")
    for name, k in [("toolX", 0), ("toolY", 1), ("toolZ", 2)]:
        v = R[:, k]
        print(f"  {name} = {[round(x, 3) for x in v]}  {axis_label(v)}  downness {downness(v):+.3f}")
    # Identify which tool column is the approach axis: the one pointing world-down.
    approach_col = max(range(3), key=lambda k: downness(R[:, k]))
    print(f"  => approach axis (most-down column) = tool-{'XYZ'[approach_col]}\n")

    def report(label, pose):
        Rp = fk(pose)[:3, :3]
        col = Rp[:, approach_col]
        print(f"{label} pose {pose}:")
        print(f"  approach(tool-{'XYZ'[approach_col]}) = {[round(x,3) for x in col]}"
              f"  tilt_from_down {tilt_deg(col):4.1f}deg toward {axis_label(col) if tilt_deg(col)>5 else '(down)'}")

    print("Single-joint sweeps from the straight-down pose (compare to hardware):")
    report("  J6=+60 (expect: spin only, tilt ~0)", [0, 0, -90, 0, 90, 60])
    report("  J5=60  (expect: tilt ~30 toward +X/forward)", [0, 0, -90, 0, 60, 0])
    report("  J4=-40 (expect: tilt ~40 toward +Y/left)", [0, 0, -90, -40, 90, 0])

    print("\nIf the three sweeps match the hardware expectations, the DH chain is")
    print("correct and we can wire it into the controller.")


if __name__ == "__main__":
    main()
