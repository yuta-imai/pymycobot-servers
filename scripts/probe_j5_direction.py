#!/usr/bin/env python3
"""Determine which J5 sign points the gripper DOWN (offline, ikpy only).

We suspect the J5 sign was backwards: the depth test may have been pointing the
gripper UP, not down. This computes the gripper approach axis (tool-Z, the 3rd
column of the ikpy FK rotation matrix) in the base frame for a sweep of J5, from
the forward-reach pose [0,-40,-40,0,J5,0], and reports `downness`:
    +1.0 = gripper points straight DOWN (tool-Z aligned with base -Z),
    -1.0 = gripper points straight UP.

No robot motion; needs ikpy + URDF only.

Usage (robot host or anywhere with ikpy):
    python3 scripts/probe_j5_direction.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

J5_SWEEP = [90, 83, 60, 30, 0, -30, -60, -83, -90]


def main():
    from mycobot_joint_controller import MyCobotJointController
    ctrl = MyCobotJointController.__new__(MyCobotJointController)
    ctrl.ik_chain = None
    ctrl.ik_active_idx = None
    ctrl.ik_error = None
    ctrl._init_ik_chain()

    print("Forward-reach pose [0,-40,-40,0,J5,0]; tool-Z = gripper approach axis")
    print("downness: +1=straight DOWN, -1=straight UP\n")
    print(f"  {'J5':>5}  {'toolZ (base xyz)':>26}   downness   gripper points")
    best = None
    for j5 in J5_SWEEP:
        angles = [0, -40, -40, 0, j5, 0]
        frame = ctrl.ik_chain.forward_kinematics(ctrl._ikpy_full_vector(angles))
        tz = [frame[i, 2] for i in range(3)]
        norm = math.sqrt(sum(c * c for c in tz)) or 1e-9
        downness = -tz[2] / norm
        tzs = "[" + ", ".join(f"{c:+.2f}" for c in tz) + "]"
        where = "DOWN" if downness > 0.7 else ("UP" if downness < -0.7 else "sideways")
        print(f"  {j5:5d}  {tzs:>26}   {downness:+.3f}    {where}")
        if best is None or downness > best[1]:
            best = (j5, downness)
    print(f"\n  Most-down J5 in this sweep: {best[0]} (downness {best[1]:+.3f})")
    print("  -> that sign is the one to use for top-down grasps.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
