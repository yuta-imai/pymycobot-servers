#!/usr/bin/env python3
"""Verify the waza teach-and-replay loop on real hardware, before a child uses it.

The waza feature (mcp-server/src/waza-tools.ts) has never run against the arm.
Two assumptions in it are load-bearing and unverified, and both fail in the
hands of whoever is holding the arm up:

  A. captureCurrentPose() reads the angles of a limp, hand-held arm and then
     calls power_on(). That assumes power_on re-engages the servos *at the
     current position*. If the firmware instead re-engages at the last
     commanded target, the arm SNAPS out of the child's hands. It also assumes
     the servos can then hold a hand-made pose without sagging.

  B. playWaza() clamps speed to MIN_SAFE_SPEED = 20 and defaults to 40, on the
     belief that J2 stalls under its own weight below that. If 20 is still too
     low for real taught poses, playback dies with "腕が止まりました" on the
     poses a child is most likely to teach (arm out and up).

This drives the same REST endpoints the MCP server uses, so a pass here is a
pass for waza itself. Servos are re-engaged in a finally, as in the real code.

SAFETY: clear the area, keep the e-stop reachable, and support the arm whenever
prompted. Test A leaves the arm limp while you pose it.

Usage (robot host, with mycobot_api_server.py running):
    ./mycobot_server_ctl.sh start
    python3 scripts/verify_waza_teach.py                 # all tests
    python3 scripts/verify_waza_teach.py --only b        # speed sweep only
    python3 scripts/verify_waza_teach.py --speeds 20,30,40,50
"""

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

# Matches waza-tools.ts: MIN_SAFE_SPEED, opts.defaultSpeed, opts.poseTimeout,
# and the 2.0 deg tolerance passed to waitForMovement during playback.
MIN_SAFE_SPEED = 20
DEFAULT_SPEED = 40
POSE_TIMEOUT = 15.0
PLAYBACK_TOLERANCE = 2.0

# Sampling schedule, in seconds, measured from the moment power_on RETURNS.
# power_on is a blocking HTTP call that takes on the order of a second, and
# nothing can be read during it -- so the instant of re-engagement is not
# observable from here. What IS observable is the aftermath: a servo that
# re-engaged at a stale target STAYS there, so a first sample equal to the
# captured pose rules that failure out even though the transient was missed.
HOLD_SAMPLES = (0.0, 0.3, 1.0, 2.0)

# Droop is sampled separately, after the operator lets go. While a human is
# holding the arm up, a sag measurement says nothing about the servos.
DROOP_SAMPLES = (0.0, 2.0, 5.0, 10.0)

# A snap this large is a hazard, not a measurement.
SNAP_DEG = 8.0
# Sag beyond this means the servos cannot hold a hand-made pose.
DROOP_DEG = 5.0

# From mycobot_joint_controller.py. A reading outside these is a corrupt
# frame, not a pose -- the joint cannot mechanically be there.
JOINT_LIMITS = {
    1: (-168, 168), 2: (-135, 135), 3: (-150, 150),
    4: (-145, 145), 5: (-155, 160), 6: (-180, 180),
}

NEUTRAL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# The setup move must actually land near neutral, or the row that follows is
# measuring a different move than the one it reports.
SETUP_TOL = 3.0
# Below this, the "move" had nowhere to go and says nothing about torque.
MIN_TRAVEL_DEG = 5.0

# From waza.example.json — the poses a child actually meets on day one. These
# load J2/J3 the most, so they are the honest test of the speed floor.
EXAMPLE_POSES = [
    ("ばんざい", [0, -40, 20, 0, 90, 0]),
    ("おじぎ (下)", [0, -50, 40, 0, 0, 0]),
    ("てをふる (右)", [0, -30, 10, 0, 60, -30]),
    ("てをふる (左)", [0, -30, 10, 0, 60, 30]),
]


class Api:
    def __init__(self, base):
        self.base = base.rstrip("/")

    def call(self, method, path, body=None, timeout=30):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            self.base + path,
            data=data,
            method=method,
            headers={"Content-Type": "application/json"} if data else {},
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")[:300]
            raise RuntimeError(f"{method} {path} -> HTTP {e.code}: {detail}") from None
        except urllib.error.URLError as e:
            raise RuntimeError(f"{method} {path} -> unreachable: {e.reason}") from None

    def angles(self, retries=2):
        """Read the pose, rejecting readings the arm cannot physically hold.

        A single corrupt serial frame surfaces as a joint angle outside its
        mechanical limit (a run of this test saw J2 report -164.01 deg against
        a +/-135 limit, with the arm demonstrably fine before and after). Left
        alone, one bad frame is indistinguishable from a real failure -- so
        re-read instead of reasoning about a value the joint cannot occupy.
        """
        for attempt in range(retries + 1):
            angles = self.call("GET", "/joints/angles")["angles"]
            bad = [
                (i + 1, a)
                for i, a in enumerate(angles)
                if not (JOINT_LIMITS[i + 1][0] <= a <= JOINT_LIMITS[i + 1][1])
            ]
            if not bad:
                return angles
            print(
                f"  ! 可動域外の読み {bad} — 読み直します "
                f"({attempt + 1}/{retries + 1})",
                file=sys.stderr,
            )
            time.sleep(0.2)
        return angles  # exhausted retries: hand it back, callers still check

    def move_all(self, angles, speed):
        return self.call("PUT", "/joints/angles", {"angles": angles, "speed": int(speed)})

    def wait(self, target=None, timeout=POSE_TIMEOUT, tolerance=PLAYBACK_TOLERANCE):
        body = {"timeout": timeout, "tolerance": tolerance}
        if target is not None:
            body["target"] = target
        return self.call("POST", "/robot/wait", body, timeout=timeout + 15)

    def release(self):
        return self.call("POST", "/robot/release")

    def power_on(self):
        return self.call("POST", "/robot/power_on")


def fmt(angles):
    return "[" + ", ".join(f"{a:7.2f}" for a in angles) + "]"


def deltas(a, b):
    return [x - y for x, y in zip(a, b)]


def worst(d):
    """Largest absolute deviation and which joint it is on (1-indexed)."""
    i = max(range(len(d)), key=lambda k: abs(d[k]))
    return i + 1, d[i]


def confirm(prompt):
    try:
        return input(f"\n>>> {prompt} [Enter=続行 / q=中止] ").strip().lower() != "q"
    except (EOFError, KeyboardInterrupt):
        return False


# --------------------------------------------------------------------- test A


def test_teach_and_hold(api, results):
    """release -> pose by hand -> read -> power_on -> does it snap or sag?

    Two separate questions, and they need different conditions to answer:

      snap  - does re-engaging throw the arm out of the holder's hands? Asked
              while they are still supporting it, because that is the moment
              under test.
      droop - can the servos hold a hand-made pose? Cannot be asked while a
              human is holding the arm: they, not the servos, would be the
              thing being measured. So we ask them to let go first.
    """
    print("\n" + "=" * 72)
    print("TEST A: 手で教えたポーズを保持できるか (captureCurrentPose の前提)")
    print("=" * 72)
    print(
        "腕の力が抜けます。両手でしっかり支えてください。\n"
        "支えたまま、子どもが教えそうな『腕を前に出して上げた』形を作ってください。"
    )
    if not confirm("腕を支えましたか？ 支えてから続けてください"):
        results.append(("A", "SKIP", "操作者が中止"))
        return None

    api.release()
    print("サーボを解放しました。手で形を作ってください（腕は支えたまま）。")

    captured = None
    samples = []
    droop = []
    hands_off = False
    power_on_s = float("nan")
    try:
        if not confirm("形ができたら続けてください（腕はまだ離さない）"):
            results.append(("A", "SKIP", "操作者が中止"))
            return None

        captured = api.angles()
        print(f"\n記録した角度  {fmt(captured)}")
        print("サーボを入れなおします。腕はまだ支えたままにしてください。")

        # Time the call itself: it is a blind window, and its length is the
        # honest bound on how fast this test can possibly see anything.
        t_call = time.monotonic()
        api.power_on()
        power_on_s = time.monotonic() - t_call

        t0 = time.monotonic()  # sampling starts when power_on RETURNS
        for target_t in HOLD_SAMPLES:
            while time.monotonic() - t0 < target_t:
                time.sleep(0.01)
            samples.append((time.monotonic() - t0, api.angles()))

        print(f"\npower_on() の所要 {power_on_s:.2f}s （この間は読めません）")
        print(f"{'経過':>6}  {'角度':<56} {'最大ずれ':>10}")
        for t, ang in samples:
            j, d = worst(deltas(ang, captured))
            print(f"{t:5.2f}s  {fmt(ang)}  J{j} {d:+6.2f}°")

        snap_j, snap_d = worst(deltas(samples[0][1], captured))
        if abs(snap_d) >= SNAP_DEG:
            print("\n跳ねを検出しました。手を離す段階には進みません。")
        else:
            print(
                "\n次はサーボの保持力を測ります。手を離す必要があります。"
                "\n（支えたままだと、測れるのは手の安定性であってサーボではありません）"
            )
            hands_off = confirm("腕から手を離してください。離したら続けてください")
            if hands_off:
                t1 = time.monotonic()
                for target_t in DROOP_SAMPLES:
                    while time.monotonic() - t1 < target_t:
                        time.sleep(0.01)
                    droop.append((time.monotonic() - t1, api.angles()))

                print(f"\n{'経過':>6}  {'角度':<56} {'最大ずれ':>10}")
                for t, ang in droop:
                    j, d = worst(deltas(ang, captured))
                    print(f"{t:5.2f}s  {fmt(ang)}  J{j} {d:+6.2f}°")
    finally:
        # Same guarantee as captureCurrentPose(): never leave the arm limp.
        try:
            api.power_on()
        except Exception as e:  # noqa: BLE001 - report, never mask the test result
            print(f"!! サーボ再投入に失敗: {e}", file=sys.stderr)

    if not samples:
        results.append(("A", "SKIP", "サンプルが取れませんでした"))
        return captured

    snap_j, snap_d = worst(deltas(samples[0][1], captured))

    print()
    if abs(snap_d) >= SNAP_DEG:
        verdict = (
            "FAIL",
            f"power_on 復帰直後に J{snap_j} が {snap_d:+.2f}° ずれています。"
            f"古い目標値へスナップした疑い — 子どもが腕を持ったまま起きると危険。"
            f"save_waza を使わせる前に、power_on 前へ現在角度を送り込む修正が必要。",
        )
    elif not hands_off:
        verdict = (
            "PARTIAL",
            f"スナップは検出されませんでした (J{snap_j} {snap_d:+.2f}°、"
            f"power_on の所要 {power_on_s:.2f}s)。"
            f"ただし手を離していないため、サーボの保持力は未測定です。",
        )
    else:
        droop_j, droop_d = worst(deltas(droop[-1][1], captured))
        if abs(droop_d) >= DROOP_DEG:
            verdict = (
                "FAIL",
                f"手を離してから {droop[-1][0]:.0f} 秒で J{droop_j} が {droop_d:+.2f}° 垂れました。"
                f"手で作ったポーズをサーボが保持できていません。"
                f"腕を下げ気味のポーズだけに限るか、教える姿勢を見直すこと。",
            )
        else:
            verdict = (
                "PASS",
                f"スナップ J{snap_j} {snap_d:+.2f}° / "
                f"手を離して {droop[-1][0]:.0f}秒後 J{droop_j} {droop_d:+.2f}° — "
                f"どちらも許容内。教える手順はそのまま使える。",
            )

    print(f"[{verdict[0]}] {verdict[1]}")
    results.append(("A", *verdict))
    return captured


# --------------------------------------------------------------------- test B


def run_pose(api, label, target, speed, start):
    """Drive one pose exactly as playWaza does, and report what happened.

    `start` is the measured pose the move begins from. Without it a row is
    unreadable: wait_for_completion checks convergence on its FIRST poll, so a
    move whose target is already within tolerance returns "converged" in 0.00s
    having moved nothing -- and that is indistinguishable, in the reason and
    error columns alone, from a move that succeeded.
    """
    effective = max(MIN_SAFE_SPEED, speed)
    travel = max(abs(t - s) for t, s in zip(target, start))

    api.move_all(target, effective)
    w = api.wait(target=target)
    reason = w.get("reason", "?")
    elapsed = w.get("elapsed_time", float("nan"))
    max_error = w.get("max_error")
    reached = api.angles()
    j, d = worst(deltas(reached, target))

    # A move with nowhere to go proves nothing about torque at this speed.
    noop = travel < MIN_TRAVEL_DEG
    if noop:
        status = "NOOP "
    elif reason in ("stalled", "timeout"):
        status = "STALL"
    else:
        status = "ok   "

    err = f"{max_error:5.2f}" if isinstance(max_error, (int, float)) else "  n/a"
    print(
        f"  speed {effective:3d}  {status} {reason:<9} "
        f"{elapsed:5.2f}s  移動量 {travel:6.2f}°  max_error {err}°  "
        f"実測ずれ J{j} {d:+6.2f}°"
    )
    return {
        "label": label,
        "speed": effective,
        "reason": reason,
        "elapsed": elapsed,
        "travel": travel,
        "noop": noop,
        "max_error": max_error,
        "worst_joint": j,
        "worst_delta": d,
    }


def sweep_pose(api, label, target, speeds, results):
    print(f"\n--- {label}  target {fmt(target)}")
    rows = []
    setup_failures = []
    for speed in speeds:
        # Approach from a common start so each speed sees the same load path.
        api.move_all(NEUTRAL, 50)
        api.wait(target=NEUTRAL)
        start = api.angles()

        # Verify the setup actually happened. An unchecked setup move is how a
        # sweep reports four clean passes while the arm sat still the whole time.
        sj, sd = worst(deltas(start, NEUTRAL))
        if abs(sd) > SETUP_TOL:
            print(
                f"  speed {max(MIN_SAFE_SPEED, speed):3d}  SETUP 中立姿勢へ戻れず "
                f"J{sj} が {sd:+.2f}° ずれたまま  ({fmt(start)})"
            )
            setup_failures.append(speed)
            continue

        rows.append(run_pose(api, label, target, speed, start))

    stalled = [r for r in rows if r["reason"] in ("stalled", "timeout")]
    noops = [r for r in rows if r["noop"]]

    if setup_failures:
        results.append(
            (
                "B:" + label,
                "FAIL",
                f"speed {setup_failures} の前に中立姿勢へ戻れませんでした。"
                f"この姿勢の到達性は測定できていません。",
            )
        )
    elif stalled:
        floor = max(r["speed"] for r in stalled)
        results.append(
            (
                "B:" + label,
                "FAIL",
                f"speed {', '.join(str(r['speed']) for r in stalled)} で停止。"
                f"MIN_SAFE_SPEED を {floor} より上へ（{floor + 10} 目安）上げる必要がある。",
            )
        )
    elif noops:
        results.append(
            (
                "B:" + label,
                "PARTIAL",
                f"speed {', '.join(str(r['speed']) for r in noops)} は移動量が "
                f"{MIN_TRAVEL_DEG}° 未満で、トルクを試していません。",
            )
        )
    else:
        results.append(("B:" + label, "PASS", f"全速度で到達 (移動量 {rows[0]['travel']:.0f}°)"))
    return rows


def test_speed_floor(api, taught, speeds, results):
    print("\n" + "=" * 72)
    print("TEST B: 速度ごとの到達性 (MIN_SAFE_SPEED=20 / 既定 40 の妥当性)")
    print("=" * 72)
    print("腕が動きます。周囲を空けて、非常停止に手を届かせてください。")
    if not confirm("動かしてよいですか？"):
        results.append(("B", "SKIP", "操作者が中止"))
        return []

    poses = list(EXAMPLE_POSES)
    if taught is not None:
        poses.insert(0, ("教えたポーズ", [round(a, 2) for a in taught]))

    rows = []
    try:
        for label, target in poses:
            rows += sweep_pose(api, label, target, speeds, results)
    finally:
        try:
            api.move_all(NEUTRAL, 50)
            api.wait(target=NEUTRAL)
        except Exception as e:  # noqa: BLE001
            print(f"!! 中立姿勢への復帰に失敗: {e}", file=sys.stderr)

    if rows:
        # Only rows that actually travelled count as evidence about a speed.
        proven = sorted({
            r["speed"] for r in rows
            if not r["noop"] and r["reason"] not in ("stalled", "timeout")
        })
        skipped = sorted({r["speed"] for r in rows if r["noop"]})
        print(f"\n実際に移動して到達できた速度: {proven or 'なし'}")
        if skipped:
            print(f"移動量不足で未検証の速度: {skipped}")
        print(
            f"既定 {DEFAULT_SPEED} は "
            f"{'安全' if DEFAULT_SPEED in proven else '未検証 — 上の行を確認してください'}"
        )
    return rows


# ------------------------------------------------------------------------ main


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--only", choices=["a", "b"], help="片方のテストだけ実行")
    p.add_argument(
        "--speeds",
        default="20,30,40,50",
        help="TEST B で試す速度 (既定: 20,30,40,50)",
    )
    args = p.parse_args()

    speeds = [int(s) for s in args.speeds.split(",") if s.strip()]
    api = Api(f"http://{args.host}:{args.port}")

    try:
        health = api.call("GET", "/health", timeout=5)
    except RuntimeError as e:
        print(f"APIサーバーに届きません: {e}\n  ./mycobot_server_ctl.sh start を先に実行してください.", file=sys.stderr)
        return 2
    print(f"API OK: {json.dumps(health, ensure_ascii=False)}")
    print(f"現在角度  {fmt(api.angles())}")

    results = []
    taught = None
    try:
        if args.only != "b":
            taught = test_teach_and_hold(api, results)
        if args.only != "a":
            test_speed_floor(api, taught, speeds, results)
    except KeyboardInterrupt:
        print("\n中断されました。サーボを入れなおします。", file=sys.stderr)
        try:
            api.power_on()
        except Exception:  # noqa: BLE001
            pass
        return 130

    print("\n" + "=" * 72)
    print("まとめ")
    print("=" * 72)
    for name, status, note in results:
        print(f"[{status:7}] {name}: {note}")

    failed = [r for r in results if r[1] == "FAIL"]
    if failed:
        print(f"\n{len(failed)} 件 FAIL。子どもに触らせる前に直してください。")
        return 1

    # PARTIAL is not a pass. It means the measurement was not actually taken,
    # and reporting it as green is how an unverified assumption reaches a child.
    partial = [r for r in results if r[1] == "PARTIAL"]
    if partial:
        print(f"\n{len(partial)} 件が測定未完了です。合格ではありません:")
        for name, _, note in partial:
            print(f"  - {name}: {note}")
        return 1
    if any(r[1] == "SKIP" for r in results):
        print("\nスキップあり。全項目を通してから本番に使ってください。")
        return 0
    print("\n全項目 PASS。waza の教える→再生ループは実機で成立しています。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
