#!/usr/bin/env python3
"""Read and (carefully) rewrite the ID of a joint servo.

Why this needs gates: the six joint servos share one half-duplex bus and are
addressed by ID. A write goes to whoever is listening on that ID -- so if two
servos both answer to ID 1 (the usual result of fitting a replacement servo
without re-assigning it), then "set ID 1 -> 2" is accepted by BOTH and you end
up with two servos on ID 2. The fault moves, it does not go away. There is no
way to single out one of two colliding servos in software.

So the only safe write is one made with exactly one servo on the bus, and this
tool refuses to write until it has confirmed that itself.

PHYSICAL SETUP before --set-id:
  1. Power off the arm.
  2. Unplug the servo chain from the controller and connect the controller's
     servo cable DIRECTLY to the servo you are re-assigning, with nothing
     downstream of it. Going through the arm's own controller is what lets
     pymycobot talk to it at all.
  3. Power on, then run --scan and confirm exactly ONE responder.
  4. Only then --set-id.
  5. Power off, rebuild the chain, run --scan again to confirm 1..6.

Address 5 as the ID register comes from the Feetech/STS memory table. Confirm
it on your own servo with --dump before trusting it -- --dump is read-only.

The serial port has one owner, so stop the API server first:
    ./mycobot_server_ctl.sh stop

    python3 scripts/servo_id_tool.py --scan
    python3 scripts/servo_id_tool.py --dump 1
    python3 scripts/servo_id_tool.py --set-id 1:2
"""

import argparse
import sys
import time

# Feetech/STS memory table.
ADDR_ID = 5
ADDR_LOCK = 55  # 0 = EEPROM writable, 1 = locked. ID lives in EEPROM.

# pymycobot range-checks servo_no; 1..6 is what the arm's firmware exposes.
SCAN_IDS = range(1, 7)

READ_REPEATS = 3


def read_reg(mc, sid, addr):
    """Read one register. None means no usable answer.

    Two servos on one ID both drive the bus, so their replies collide and this
    comes back as None or as a value that changes between calls -- which is why
    the caller reads more than once.
    """
    fn = getattr(mc, "get_servo_data", None)
    if fn is None:
        raise RuntimeError("このpymycobotには get_servo_data がありません")
    for call_args in ((sid, addr), (sid, addr, 1)):
        try:
            return fn(*call_args)
        except TypeError:
            continue  # signature differs between pymycobot versions
        except Exception:
            return None
    return None


def write_reg(mc, sid, addr, value):
    fn = getattr(mc, "set_servo_data", None)
    if fn is None:
        raise RuntimeError("このpymycobotには set_servo_data がありません")
    for call_args in ((sid, addr, value), (sid, addr, value, 1)):
        try:
            fn(*call_args)
            return True
        except TypeError:
            continue
        except Exception as e:  # noqa: BLE001
            print(f"  書き込み例外: {type(e).__name__}: {e}", file=sys.stderr)
            return False
    return False


def plausible(v):
    """Is this a real register byte, or an error sentinel?

    pymycobot signals a failed read by RETURNING -1, not by raising or
    returning None. Counting that as an answer is how a bus with one servo
    attached reported six: every absent ID answered -1, identically, which a
    naive "not None and consistent" test reads as a healthy stable servo.
    """
    return isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 255


def probe(mc, sid, repeats=READ_REPEATS):
    """Classify one ID from its own ID register.

    A servo addressed at ID n necessarily holds n at ADDR_ID -- that is what
    being ID n means. So the read must come back equal to sid to count as a
    presence. Anything else is a sentinel, bus noise, or a wiring surprise, and
    none of those are 'a servo is here'.

    Returns one of:
      absent    no plausible reply
      present   consistent, and equal to sid
      unstable  plausible but varying / intermittent -> two servos on one ID
      mismatch  consistent but != sid -> answering, but not who we addressed
    """
    reads = []
    for _ in range(repeats):
        reads.append(read_reg(mc, sid, ADDR_ID))
        time.sleep(0.05)

    good = [r for r in reads if plausible(r)]
    if not good:
        return "absent", reads
    if len(set(good)) > 1 or len(good) != repeats:
        return "unstable", reads
    return ("present" if good[0] == sid else "mismatch"), reads


def scan(mc, verbose=True):
    """Return {id: (state, reads)} for every ID that answers."""
    found = {}
    if verbose:
        print(f"\n--- バス走査 (ID {SCAN_IDS.start}..{SCAN_IDS.stop - 1}) ---")
    for sid in SCAN_IDS:
        state, reads = probe(mc, sid)
        if state != "absent":
            found[sid] = (state, reads)
        if verbose:
            mark = {
                "absent": "応答なし",
                "present": "応答あり",
                "unstable": "応答が不安定  <-- 同一IDに複数台いる疑い",
                "mismatch": "応答したがIDが一致しない  <-- サーボではない可能性",
            }[state]
            print(f"  ID {sid}: {mark}  reads={reads}")
    if verbose:
        present = [i for i, (s, _) in found.items() if s == "present"]
        print(f"\n応答したID: {sorted(present) or 'なし'}  (計 {len(present)} 台)")
        if len(found) != len(present):
            other = sorted(set(found) - set(present))
            print(f"判定不能なID: {other}  (reads を見て判断してください)")
    return found


def dump(mc, sid, start, end):
    """Read a slice of the memory table so the ID address can be confirmed."""
    print(f"\n--- ID {sid} のメモリテーブル {start}..{end} ---")
    print("書き換え前に、どのアドレスが実際にIDを保持しているか確認してください。")
    for addr in range(start, end + 1):
        v = read_reg(mc, sid, addr)
        tag = ""
        if addr == ADDR_ID:
            tag = "  <-- ID とみなしているアドレス"
        elif addr == ADDR_LOCK:
            tag = "  <-- EEPROMロック (0=書込可 / 1=ロック)"
        print(f"  addr {addr:3d} = {v}{tag}")
        time.sleep(0.02)


def set_id(mc, old, new, force, skip_lock):
    print("\n" + "=" * 70)
    print(f"サーボID書き換え:  {old}  ->  {new}")
    print("=" * 70)

    found = scan(mc)

    # Gate 1: exactly one servo on the bus. This is the whole safety argument.
    # Counts responders of ANY state, not just clean ones: an ID we cannot
    # classify is a possible servo, and possible servos block the write.
    if len(found) != 1 or old not in found:
        print(
            f"\n中止: バス上に {len(found)} 台 (ID {sorted(found)}) 見えています。\n"
            f"書き換えは、対象の1台だけがバスに繋がっている状態でしか安全に行えません。\n"
            f"複数台いる状態で書くと全部が同じIDを受理します。\n"
            f"コントローラのケーブルを対象サーボに直結し、他を外してからやり直してください。"
        )
        if not force:
            return 1
        print("\n--force が指定されています。ゲートを飛ばします。")

    # Gate 2: the one responder must be cleanly identified as ID `old`.
    # Unstable = two servos answering here; mismatch = not the servo we think.
    if old in found and found[old][0] != "present" and not force:
        state, reads = found[old]
        print(
            f"\n中止: ID {old} の状態が「{state}」です (reads={reads})。\n"
            f"  unstable -> まだ複数台が同じIDで応答しています。物理的に1台だけにしてください。\n"
            f"  mismatch -> 応答はあるがIDレジスタが一致しません。--dump {old} で確認してください。\n"
            f"確実に同定できない相手にIDを書き込むことはしません。"
        )
        return 1

    # Gate 3: never create a second collision at the destination.
    if new in found:
        print(f"\n中止: ID {new} には既に別のサーボが応答しています。衝突を新たに作ることになります。")
        return 1

    # Gate 4: a deliberate, typed confirmation -- not a stray Enter.
    print(
        f"\nID {old} のサーボに、新しいID {new} を書き込みます（EEPROM、電源を切っても残ります）。"
    )
    try:
        typed = input(f">>> 実行するなら {new} と入力してください [その他=中止] ").strip()
    except (EOFError, KeyboardInterrupt):
        return 130
    if typed != str(new):
        print("中止しました。")
        return 1

    if not skip_lock:
        print(f"\nEEPROMロックを解除 (addr {ADDR_LOCK} = 0)")
        write_reg(mc, old, ADDR_LOCK, 0)
        time.sleep(0.1)

    print(f"ID を書き込み (addr {ADDR_ID} = {new})")
    ok = write_reg(mc, old, ADDR_ID, new)
    time.sleep(0.2)

    # From here the servo answers on `new`, so re-lock must be addressed there.
    if not skip_lock:
        print(f"EEPROMロックを再設定 (addr {ADDR_LOCK} = 1)")
        if not write_reg(mc, new, ADDR_LOCK, 1):
            write_reg(mc, old, ADDR_LOCK, 1)
        time.sleep(0.1)

    print("\n--- 検証 ---")
    new_state, new_reads = probe(mc, new)
    old_state, old_reads = probe(mc, old)
    print(f"  新ID {new}: {new_state}  reads={new_reads}")
    print(f"  旧ID {old}: {old_state}  reads={old_reads}")

    if new_state == "present" and old_state == "absent":
        print(f"\n成功: サーボは ID {new} で応答し、ID {old} は消えました。")
        print(
            "\n次にやること:\n"
            "  1. 電源を切り、サーボチェーンを元通りに組み直す\n"
            "  2. --scan で ID 1..6 が全部 stable で見えることを確認\n"
            "  3. この関節のゼロ点キャリブレーションを実施\n"
            "     (交換したサーボは原点を知らないため、IDが通っても角度がずれます)"
        )
        return 0

    print(
        f"\n失敗: 書き込みが反映されていません。\n"
        f"  - EEPROMがロックされたままの可能性 (--dump {old} で addr {ADDR_LOCK} を確認)\n"
        f"  - ADDR_ID={ADDR_ID} がこのサーボのIDアドレスでない可能性 (--dump で確認)\n"
        f"  - サーボが応答していない可能性 (--scan で確認)"
    )
    if not ok:
        print("  - set_servo_data の呼び出し自体が失敗しています")
    return 1


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--scan", action="store_true", help="バス上のサーボを一覧（読み取りのみ）")
    p.add_argument("--dump", type=int, metavar="ID", help="指定IDのメモリテーブルを読む（読み取りのみ）")
    p.add_argument("--dump-range", default="0:60", help="--dump の範囲 (既定 0:60)")
    p.add_argument("--set-id", metavar="OLD:NEW", help="IDを書き換える（安全ゲートあり）")
    p.add_argument("--force", action="store_true", help="1台だけという確認を飛ばす（非推奨）")
    p.add_argument("--skip-lock", action="store_true", help="EEPROMロックの解除/再設定を行わない")
    args = p.parse_args()

    if not (args.scan or args.dump is not None or args.set_id):
        args.scan = True  # read-only default

    # Validate before opening the port, so a typo never reaches the servo bus.
    old = new = None
    if args.set_id:
        try:
            old, new = (int(x) for x in args.set_id.split(":"))
        except ValueError:
            print("--set-id は OLD:NEW の形式で指定してください (例: 1:2)", file=sys.stderr)
            return 2
        if not (1 <= new <= 6):
            print(f"新しいIDは 1..6 にしてください（この腕の関節番号）。指定: {new}", file=sys.stderr)
            return 2
        if old == new:
            print(f"旧IDと新IDが同じです: {old}", file=sys.stderr)
            return 2

    try:
        from pymycobot import MyCobot
    except ImportError:
        print("pymycobot が入っていません。ロボットが繋がっているホストで実行してください。", file=sys.stderr)
        return 2

    print(f"接続: {args.port} @ {args.baudrate}")
    mc = MyCobot(args.port, args.baudrate)
    time.sleep(0.5)

    if args.dump is not None:
        start, end = (int(x) for x in args.dump_range.split(":"))
        dump(mc, args.dump, start, end)
        return 0

    if args.set_id:
        return set_id(mc, old, new, args.force, args.skip_lock)

    found = scan(mc)
    missing = [i for i in SCAN_IDS if i not in found]
    unstable = [i for i, (s, _) in found.items() if s == "unstable"]
    mismatch = [i for i, (s, _) in found.items() if s == "mismatch"]
    if unstable:
        print(f"\nID {unstable} の応答が不安定です。同じIDに複数台いる可能性が高い。")
    if mismatch:
        print(f"ID {mismatch} は応答しましたがIDレジスタが一致しません。--dump で中身を確認してください。")
    if missing:
        print(f"ID {missing} が不在です。そのIDのサーボが別のIDを名乗っているか、繋がっていません。")
    if not missing and not unstable and not mismatch:
        print("\nID 1..6 がすべて正常に応答しています。")

    print(
        "\n注意: pymycobot はサーボと直接ではなく腕のコントローラ経由で会話します。"
        "コントローラが実際にバスへ問い合わせず自分の状態から答える場合、"
        "この結果は物理的な接続を反映しません。実装が信用できるかは、"
        "1台だけ繋いだ状態の出力と全台繋いだ状態の出力を比べれば分かります。"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
