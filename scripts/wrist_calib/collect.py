#!/usr/bin/env python3
"""Calibration data collector — RUNS ON THE ROBOT HOST (drives /dev/ttyACM0).

For each pose: move (joint-limit checked) -> settle -> read back joint angles +
get_coords + a STABLE gravity sample from the sensor -> append one JSONL record.
The gravity sensor is injected via the GravitySource interface (gravity_source.py),
so this works with the BLE sensor once it exists, and with the manual/replay sources
today. Output feeds calibrate.py.

Record schema (one JSON object per line):
  {"i", "commanded":[6], "angles":[6], "coords":[6], "gravity":[gx,gy,gz],
   "still":bool, "std_ratio":float}

Safety: never calls firmware send_coords/go_home. Clear the workspace; keep e-stop
in reach. --dry-run exercises the sensor + pose list WITHOUT moving the arm.

Usage (robot host, project venv):
  python3 scripts/wrist_calib/collect.py --provider manual --pose-set quick --dry-run
  python3 scripts/wrist_calib/collect.py --provider ble --ble-name MYSENSOR \
        --pose-set default --out calib_$(date +%s).jsonl --speed 20
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import poses as poselib            # noqa: E402
from gravity_source import make_source  # noqa: E402

LIMITS = poselib.LIMITS


def within_limits(p):
    return all(LIMITS[i][0] <= a <= LIMITS[i][1] for i, a in enumerate(p, 1))


def build_provider(args):
    if args.provider == "ble":
        return make_source("ble", name=args.ble_name, address=args.ble_address,
                           mode=args.ble_mode, char_uuid=args.ble_char)
    return make_source(args.provider)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--provider", default="manual",
                    choices=["manual", "ble", "replay"])
    # This WT9011DCL advertises as "WT901BLE67"; connecting by address is most
    # reliable (discovered MAC: F7:50:70:EE:0D:DF).
    ap.add_argument("--ble-name")
    ap.add_argument("--ble-address")
    ap.add_argument("--ble-mode", default="witmotion",
                    choices=["witmotion", "nus", "beacon"])
    ap.add_argument("--ble-char", default=None,
                    help="notify char UUID; default per mode (WitMotion=FFE4)")
    ap.add_argument("--pose-set", default="default",
                    choices=["default", "quick"])
    ap.add_argument("--step", type=int, default=30, help="sweep step (deg)")
    ap.add_argument("--out", default=None, help="output JSONL (default: stdout note)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--speed", type=int, default=20)
    ap.add_argument("--settle", type=float, default=3.0, help="post-move settle (s)")
    ap.add_argument("--samples", type=int, default=12,
                    help="gravity samples/pose (paced ~10Hz to the sensor)")
    ap.add_argument("--still-dps", type=float, default=2.5,
                    help="stillness gate: median gyro (deg/s) below this = still")
    ap.add_argument("--retries", type=int, default=2,
                    help="re-read (with extra settle) if not still")
    ap.add_argument("--dry-run", action="store_true",
                    help="don't move the arm; just exercise sensor + poses")
    args = ap.parse_args()

    pset = poselib.default_set(args.step) if args.pose_set == "default" else poselib.quick_set()
    pset = [p for p in pset if within_limits(p)]
    print(f"[collect] {len(pset)} poses, provider={args.provider}, dry_run={args.dry_run}")

    provider = build_provider(args)

    ctrl = None
    if not args.dry_run:
        from mycobot_joint_controller import MyCobotJointController
        ctrl = MyCobotJointController(args.port, args.baudrate)
        if ctrl.ik_chain is None:
            print(f"[collect] WARNING ik_chain unavailable: {ctrl.ik_error}")
        ip = getattr(ctrl.mc, "is_power_on", lambda: 1)()
        if ip != 1:
            print("[collect] servos not powered; aborting."); return 2

    out = open(args.out, "w") if args.out else None
    n_ok = 0
    try:
        for i, p in enumerate(pset):
            print(f"[{i+1}/{len(pset)}] pose {[round(x) for x in p]}", flush=True)
            angles = list(p)
            coords = [None] * 6
            if not args.dry_run:
                ctrl.mc.send_angles(p, args.speed)
                time.sleep(args.settle)
                try:
                    angles = [round(a, 2) for a in ctrl.get_all_joint_angles()]
                    coords = [round(v, 1) for v in ctrl._read_coords(retries=3)]
                except Exception as e:
                    print(f"    read-back failed: {e}")
            try:
                g, still, motion = provider.read_stable(
                    n=args.samples, gyro_still_dps=args.still_dps)
                tries = 0
                while not still and tries < args.retries:
                    tries += 1
                    time.sleep(0.8)  # let a jitter burst pass
                    g, still, motion = provider.read_stable(
                        n=args.samples, gyro_still_dps=args.still_dps)
            except KeyboardInterrupt:
                print("    gravity skipped"); continue
            except Exception as e:
                print(f"    gravity read failed: {e}"); continue
            rec = {"i": i, "commanded": [round(x, 2) for x in p], "angles": angles,
                   "coords": coords, "gravity": [round(v, 5) for v in g],
                   "still": bool(still), "motion": round(motion, 4)}
            line = json.dumps(rec)
            if out:
                out.write(line + "\n"); out.flush()
            print(f"    g={rec['gravity']} still={still} motion={motion:.3f}"
                  f" {'(gyro dps)' if hasattr(provider, 'gyro_magnitude') else '(accel std)'}")
            n_ok += 1
        # park
        if not args.dry_run:
            print("[collect] parking home"); ctrl.mc.send_angles([0, 0, 0, 0, 0, 0], args.speed)
            time.sleep(2.0)
    finally:
        provider.close()
        if out:
            out.close()
    print(f"[collect] recorded {n_ok} poses" + (f" -> {args.out}" if args.out else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
