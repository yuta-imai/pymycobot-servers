#!/usr/bin/env python3
"""Introspect the installed pymycobot against the connected robot.

Run this ON THE MACHINE attached to the robot to discover the actual library
version and which methods exist. We hit AttributeErrors (e.g. get_angle,
get_gripper_value) because the official docs describe APIs not present in every
pymycobot version/model — this prints the ground truth so fixes target real
methods instead of the docs.

Usage:
    python3 scripts/introspect_pymycobot.py [--port /dev/ttyACM0] [--baudrate 115200]

It only READS (version, dir(), get_angles); it sends no motion commands.
"""

import argparse
import sys

# Methods this project calls or considered calling; we report which exist.
METHODS_OF_INTEREST = [
    # angle read/write
    "get_angle", "get_angles", "send_angle", "send_angles", "sync_send_angles",
    # jog
    "jog_angle", "jog_increment_angle", "jog_stop",
    # status / control
    "is_moving", "is_in_position", "stop", "pause", "resume",
    "set_fresh_mode", "get_fresh_mode", "get_error_information",
    "power_on", "power_off", "release_all_servos", "focus_servo",
    # gripper
    "set_gripper_state", "set_gripper_value", "get_gripper_value",
    "is_gripper_moving", "set_gripper_calibration", "get_gripper_status",
    "set_gripper_mode", "get_gripper_mode",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--no-connect", action="store_true",
                        help="Only inspect the class, do not open the serial port")
    args = parser.parse_args()

    import pymycobot
    from pymycobot import MyCobot

    print(f"pymycobot version: {getattr(pymycobot, '__version__', 'unknown')}")
    print(f"pymycobot path:    {pymycobot.__file__}")
    print()

    # Class-level method presence (no hardware needed).
    print("== Methods of interest (on MyCobot class) ==")
    for name in METHODS_OF_INTEREST:
        present = hasattr(MyCobot, name)
        print(f"  {'OK ' if present else 'NO '} {name}")
    print()

    if args.no_connect:
        return 0

    print(f"Connecting to {args.port} @ {args.baudrate} ...")
    mc = MyCobot(args.port, args.baudrate)

    print("\n== Public attributes on the connected instance ==")
    public = sorted(m for m in dir(mc) if not m.startswith("_"))
    print("  " + ", ".join(public))

    print("\n== Safe read probes ==")
    for name in ("get_angles", "is_moving", "get_error_information",
                 "get_gripper_value", "is_gripper_moving"):
        fn = getattr(mc, name, None)
        if fn is None:
            print(f"  {name}: <absent>")
            continue
        try:
            print(f"  {name}() -> {fn()!r}")
        except Exception as exc:  # noqa: BLE001 - report whatever happens
            print(f"  {name}() raised {type(exc).__name__}: {exc}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
