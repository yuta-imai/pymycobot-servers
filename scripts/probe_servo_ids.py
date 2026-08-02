#!/usr/bin/env python3
"""Find out which servo IDs are actually answering on the joint bus.

Written for this symptom, after a J2 servo replacement:
  - a command to J1 moves BOTH J1 and J2
  - J2 cannot be addressed at all

The six joint servos share one half-duplex serial bus and are addressed by ID
1-6. A replacement servo ships with a factory default ID (commonly 1). If the
new J2 servo was never re-assigned to ID 2, then:
  - packets addressed to ID 1 are accepted by two servos, so both move
  - nothing answers at ID 2
which is exactly the pair of symptoms above. No software fault can produce it:
move_joint() passes joint_num straight to send_angle(), so the J1 and J2 paths
are byte-identical (mycobot_joint_controller.py:243).

This script only READS by default -- no motion, and it never writes a servo ID.
Writing an ID while two servos share one is actively harmful: BOTH accept the
write, so "set ID 1 -> 2" leaves you with two ID-2 servos and the same fault
mirrored onto J2. The new servo has to be isolated on the bus first.

The serial port allows one owner, so stop the API server first:
    ./mycobot_server_ctl.sh stop
    python3 scripts/probe_servo_ids.py
    python3 scripts/probe_servo_ids.py --limp-test   # decisive, needs a helper
"""

import argparse
import sys
import time

# Feetech/STS serial-bus servo memory table: address 5 holds the servo ID.
# Confirm against your servo's datasheet before ever writing to it.
ADDR_ID = 5

JOINT_NAMES = {
    1: "J1 (base)",
    2: "J2 (shoulder)",
    3: "J3 (elbow)",
    4: "J4 (wrist 1)",
    5: "J5 (wrist 2)",
    6: "J6 (wrist 3)",
}


def call(mc, name, *args):
    """Call a pymycobot method if this version has it. Returns (ok, value)."""
    fn = getattr(mc, name, None)
    if fn is None:
        return False, f"(このpymycobotに {name} はありません)"
    try:
        return True, fn(*args)
    except Exception as e:  # noqa: BLE001 - a raising probe is itself a result
        return False, f"(例外: {type(e).__name__}: {e})"


def probe_enable(mc):
    """Which IDs answer 'yes, I am here'? A missing ID is the whole story."""
    print("\n--- is_servo_enable(1..6): バス上で応答するサーボ ---")
    if getattr(mc, "is_servo_enable", None) is None:
        print("  is_servo_enable がありません。スキップします。")
        return {}

    seen = {}
    for i in range(1, 7):
        ok, v = call(mc, "is_servo_enable", i)
        seen[i] = v if ok else None
        if not ok:
            note = str(v)
        elif v == 1:
            note = "応答あり"
        elif v == 0:
            note = "応答なし  <-- ここにサーボがいない"
        else:
            note = f"不定 (戻り値 {v})"
        print(f"  ID {i}  {JOINT_NAMES[i]:<16} -> {str(v):<6} {note}")
    return seen


def probe_id_register(mc, repeats=3):
    """Read each servo's own ID register, several times.

    With two servos on one ID, both drive the bus at once on every read. The
    replies collide, so the answer comes back as None, garbage, or a value that
    changes between reads -- inconsistency here is itself the evidence.
    """
    print(f"\n--- get_servo_data(id, {ADDR_ID}): 各IDのIDレジスタを{repeats}回読む ---")
    if getattr(mc, "get_servo_data", None) is None:
        print("  get_servo_data がありません。スキップします。")
        return

    for i in range(1, 7):
        reads = []
        for _ in range(repeats):
            ok, v = call(mc, "get_servo_data", i, ADDR_ID)
            reads.append(v if ok else None)
            time.sleep(0.05)

        distinct = {r for r in reads if r is not None}
        if not distinct:
            note = "応答なし"
        elif len(distinct) > 1:
            note = "読むたびに違う  <-- 複数のサーボが同時に返答している疑い"
        elif reads.count(next(iter(distinct))) != repeats:
            note = "応答が不安定  <-- バス競合の疑い"
        elif next(iter(distinct)) != i:
            note = f"IDレジスタが {next(iter(distinct))} を返した  <-- 不一致"
        else:
            note = "一貫"
        print(f"  ID {i}  reads={reads}  {note}")


def probe_health(mc):
    """Per-servo lists (voltage/temperature/status) enumerate the whole bus."""
    print("\n--- サーボごとの状態（欠けているIDが見える） ---")
    for name in ("get_servo_voltages", "get_servo_temps", "get_servo_status"):
        ok, v = call(mc, name)
        print(f"  {name:<20} -> {v}")

    ok, v = call(mc, "get_angles")
    print(f"  {'get_angles':<20} -> {v}")
    if ok and isinstance(v, list) and len(v) >= 2:
        print(
            "     J1 と J2 の値が一緒に動くなら、2つの関節が同じ指令を受けています。"
        )


def limp_test(mc):
    """Release ID 1 alone and see how many joints go limp.

    This needs no library feature beyond release_servo, and it settles the
    question physically: if releasing ID 1 also drops the shoulder, then the
    shoulder servo is answering to ID 1.
    """
    if getattr(mc, "release_servo", None) is None:
        print("\nrelease_servo がないため limp-test は実行できません。")
        return

    print("\n" + "=" * 68)
    print("LIMP TEST: ID 1 だけ脱力させ、いくつの関節が落ちるか見る")
    print("=" * 68)
    print(
        "肩(J2)まで脱力する可能性があるため、腕は必ず両手で支えてください。\n"
        "支える人がいない状態では絶対に実行しないでください。"
    )
    try:
        if input("\n>>> 腕を支えましたか？ [Enter=続行 / q=中止] ").strip().lower() == "q":
            return
    except (EOFError, KeyboardInterrupt):
        return

    try:
        call(mc, "release_servo", 1)
        print("\nID 1 を脱力しました。手で各関節を揺すって確かめてください。")
        print("  J1 だけ動く            -> IDは正常。原因は別（配線・キャリブレーション）")
        print("  J1 と J2 の両方が動く  -> ID衝突が確定。新しいJ2サーボのIDが 1 のまま")
        input(">>> 確認したら Enter（サーボを入れなおします） ")
    finally:
        ok, _ = call(mc, "focus_servo", 1)
        if not ok:
            call(mc, "power_on")
        print("サーボを入れなおしました。")


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument(
        "--limp-test",
        action="store_true",
        help="ID 1 を脱力させて、落ちる関節の数を目視で確認する（介助者が必要）",
    )
    args = p.parse_args()

    try:
        from pymycobot import MyCobot
    except ImportError:
        print("pymycobot が入っていません。ロボットが繋がっているホストで実行してください。", file=sys.stderr)
        return 2

    print(f"接続: {args.port} @ {args.baudrate}")
    mc = MyCobot(args.port, args.baudrate)
    time.sleep(0.5)

    ok, v = call(mc, "get_system_version")
    print(f"firmware: {v}")

    seen = probe_enable(mc)
    probe_id_register(mc)
    probe_health(mc)

    print("\n" + "=" * 68)
    print("読み方")
    print("=" * 68)
    missing = [i for i, v in seen.items() if v == 0]
    if missing:
        print(
            f"ID {missing} が応答していません。"
            "そのIDのサーボが物理的にいないか、別のIDを名乗っています。"
        )
        if 2 in missing:
            print(
                "  ID 2 が不在で、かつ J1 の指令で2関節動くなら、"
                "新しいJ2サーボが ID 1 のまま出荷時設定である可能性が最も高い。"
            )
    else:
        print("1..6 すべて応答しています。ID衝突以外の原因（配線・キャリブレーション）を疑ってください。")
    print("いずれの場合も、このスクリプトは ID を書き換えません。手順は応答を見てから決めてください。")

    if args.limp_test:
        limp_test(mc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
