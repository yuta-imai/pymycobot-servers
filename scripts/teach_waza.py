#!/usr/bin/env python3
"""Teach waza by hand and write them out from what the arm actually measured.

The angles in waza.example.json were authored without ever seeing the robot,
and on hardware all four "motions" turned out to share the same silhouette:
between ばんざい, おじぎ and てをふる the shoulder and elbow (J2/J3) differ by
0-30 deg, and nearly all of the remaining difference sits in J5/J6 -- wrist
twist, which does not change the shape of the arm at all.

That is fatal for what these motions are for. A child is supposed to watch the
model read their own sentence and pick the right motion; if every motion looks
the same, there is nothing to see and no way to score a guess. So: stop
authoring joint angles, and capture them from an arm someone posed by hand --
the flow scripts/verify_waza_teach.py already proved works on this hardware.

Before writing, the captured motions are compared to each other on J2/J3 and
anything too similar to tell apart is reported, because that is the failure
this script exists to prevent.

SAFETY: the arm goes limp while you pose it. Support it with both hands, and
keep the e-stop reachable. Servos are re-engaged in a finally.

Usage (robot host, with mycobot_api_server.py running):
    python3 scripts/teach_waza.py
    python3 scripts/teach_waza.py --output waza.json --replay
"""

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent

# Mirrors ANGLE_LIMITS in mcp-server/src/waza.ts, which rejects out-of-range
# entries at load. Catching it here means the teacher finds out while the arm
# is in front of them, not when a motion silently vanishes at startup.
JOINT_LIMITS = {
    1: (-168, 168), 2: (-135, 135), 3: (-150, 150),
    4: (-145, 145), 5: (-155, 160), 6: (-180, 180),
}

MAX_POSES = 20
MAX_REPEAT = 5
MAX_HOLD_MS = 5000
DEFAULT_SPEED = 40
MIN_SAFE_SPEED = 20

# J2/J3 spread below which two motions read as the same shape from across a
# room. Derived from the failure this script exists to prevent: the fabricated
# examples sat at 0-30 deg apart and were indistinguishable.
SILHOUETTE_MIN_DEG = 25.0


class Api:
    def __init__(self, base):
        self.base = base.rstrip("/")

    def call(self, method, path, body=None, timeout=30):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            self.base + path, data=data, method=method,
            headers={"Content-Type": "application/json"} if data else {},
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"{method} {path} -> HTTP {e.code}: "
                               f"{e.read().decode(errors='replace')[:300]}") from None
        except urllib.error.URLError as e:
            raise RuntimeError(f"{method} {path} -> unreachable: {e.reason}") from None

    def angles(self, retries=2):
        """Read the pose, rejecting values the arm cannot physically hold.

        A corrupt serial frame shows up as an angle outside its mechanical
        limit (a test run saw J2 report -164 deg against a +/-135 limit). A bad
        frame recorded here would be saved as a motion, so re-read instead.
        """
        for attempt in range(retries + 1):
            angles = self.call("GET", "/joints/angles")["angles"]
            bad = [(i + 1, a) for i, a in enumerate(angles)
                   if not (JOINT_LIMITS[i + 1][0] <= a <= JOINT_LIMITS[i + 1][1])]
            if not bad:
                return angles
            print(f"  ! 可動域外の読み {bad} — 読み直します ({attempt + 1})", file=sys.stderr)
            time.sleep(0.2)
        raise RuntimeError(f"角度が可動域外のままです: {angles}")

    def move_all(self, angles, speed):
        return self.call("PUT", "/joints/angles",
                         {"angles": angles, "speed": int(speed)})

    def wait(self, target, timeout=15.0, tolerance=2.0):
        return self.call("POST", "/robot/wait",
                         {"timeout": timeout, "tolerance": tolerance, "target": target},
                         timeout=timeout + 15)

    def release(self):
        return self.call("POST", "/robot/release")

    def power_on(self):
        return self.call("POST", "/robot/power_on")


def fmt(angles):
    return "[" + ", ".join(f"{a:7.2f}" for a in angles) + "]"


def ask(prompt, default=""):
    try:
        v = input(f"{prompt}" + (f" [{default}] " if default else " ")).strip()
    except (EOFError, KeyboardInterrupt):
        raise SystemExit("\n中断しました。")
    return v or default


def ask_int(prompt, default, lo, hi):
    while True:
        raw = ask(prompt, str(default))
        try:
            v = int(raw)
        except ValueError:
            print(f"  数字で入れてください ({lo}-{hi})")
            continue
        if lo <= v <= hi:
            return v
        print(f"  {lo}-{hi} の範囲で入れてください")


def capture_pose(api, index):
    """release -> operator poses by hand -> read -> re-engage.

    power_on is in a finally for the same reason captureCurrentPose() does it:
    someone is holding the arm up, and whether they can let go must not depend
    on this function succeeding.
    """
    print(f"\n  --- {index} 番目のポーズ ---")
    print("  腕を両手で支えてください。支えたら Enter を押すと力が抜けます。")
    ask("  支えましたか？ Enter で脱力")

    api.release()
    print("  脱力しました。手で形を作ってください（支えたまま）。")
    try:
        ask("  形ができたら Enter（腕はまだ離さない）")
        angles = api.angles()
    finally:
        try:
            api.power_on()
        except Exception as e:  # noqa: BLE001
            print(f"  !! サーボ再投入に失敗: {e}", file=sys.stderr)
    print(f"  記録: {fmt(angles)}   サーボを入れました。手を離して大丈夫です。")
    return angles


def teach_one(api):
    name = ask("\n技の名前（空 Enter で終了）:")
    if not name:
        return None
    if len(name) > 48:
        print("  48文字までです。切り詰めます。")
        name = name[:48]

    print("どんなときに使う動きか、教える人自身の言葉で書いてください。")
    print("形の説明（りょうてをあげる）より、使う場面（うれしいとき）の方がAIは選べます。")
    setsumei = ask("せつめい:")[:600]

    n = ask_int(f"ポーズをいくつ記録しますか (1-{MAX_POSES}):", 1, 1, MAX_POSES)
    poses = []
    for i in range(1, n + 1):
        angles = capture_pose(api, i)
        pose = {"angles": [round(a, 2) for a in angles]}
        if n > 1:
            hold = ask_int(f"  このポーズで止まる時間 ms (0-{MAX_HOLD_MS}):", 0, 0, MAX_HOLD_MS)
            if hold:
                pose["hold_ms"] = hold
        poses.append(pose)

    repeat = 1
    if n > 1:
        repeat = ask_int(f"何回くりかえしますか (1-{MAX_REPEAT}):", 1, 1, MAX_REPEAT)
    speed = ask_int(f"再生速度 ({MIN_SAFE_SPEED}-100):", DEFAULT_SPEED, MIN_SAFE_SPEED, 100)

    waza = {"name": name, "setsumei": setsumei, "speed": speed}
    # Single-pose motions use the flat `angles` form, as waza.example.json does.
    if len(poses) == 1 and "hold_ms" not in poses[0]:
        waza["angles"] = poses[0]["angles"]
    else:
        waza["poses"] = poses
    if repeat > 1:
        waza["repeat"] = repeat
    return waza


def first_angles(w):
    return w["angles"] if "angles" in w else w["poses"][0]["angles"]


def check_silhouettes(wazas):
    """Warn about motions a child could not tell apart.

    J2/J3 set the shape of the arm; J5/J6 only twist the wrist. Two motions
    that differ mainly in the wrist look identical from across a room, which
    defeats the point of watching a model choose between them.
    """
    if len(wazas) < 2:
        return True
    print("\n--- 見た目の違い（肩J2・肘J3 の差） ---")
    ok = True
    for i in range(len(wazas)):
        for j in range(i + 1, len(wazas)):
            a, b = first_angles(wazas[i]), first_angles(wazas[j])
            arm = max(abs(a[k] - b[k]) for k in (1, 2))
            wrist = max(abs(a[k] - b[k]) for k in (4, 5))
            mark = ""
            if arm < SILHOUETTE_MIN_DEG:
                mark = "  <-- 見分けがつきません"
                ok = False
            print(f"  {wazas[i]['name']} vs {wazas[j]['name']}: "
                  f"腕 {arm:5.1f}°  手首 {wrist:5.1f}°{mark}")
    if not ok:
        print(
            f"\n腕の差が {SILHOUETTE_MIN_DEG}° 未満の組があります。"
            "\nAIがどちらを選んだか子どもが判定できないので、教えなおすことを勧めます。"
        )
    return ok


def replay(api, wazas):
    print("\n--- 再生して確認します ---")
    for w in wazas:
        poses = w.get("poses") or [{"angles": w["angles"]}]
        speed = max(MIN_SAFE_SPEED, w.get("speed", DEFAULT_SPEED))
        print(f"\n「{w['name']}」を再生します。")
        ask("  周囲を空けて Enter")
        for r in range(w.get("repeat", 1)):
            for p in poses:
                api.move_all(p["angles"], speed)
                res = api.wait(p["angles"])
                if res.get("reason") in ("stalled", "timeout"):
                    print(f"  !! {res.get('reason')} — 速度 {speed} では届きません")
                    break
                if p.get("hold_ms"):
                    time.sleep(p["hold_ms"] / 1000.0)


def write_atomic(path, payload):
    """Temp file plus rename, so a reader never sees a half-written file."""
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    os.replace(tmp, path)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--output", default=str(HERE / "waza.example.json"))
    p.add_argument("--replay", action="store_true", help="保存前に再生して確認する")
    args = p.parse_args()

    api = Api(f"http://{args.host}:{args.port}")
    try:
        print(f"API OK: {json.dumps(api.call('GET', '/health', timeout=5), ensure_ascii=False)}")
    except RuntimeError as e:
        print(f"APIサーバーに届きません: {e}", file=sys.stderr)
        return 2
    print(f"現在角度  {fmt(api.angles())}")

    print("\n技を1つずつ手で教えます。名前を空 Enter にすると終了します。")
    wazas = []
    while True:
        w = teach_one(api)
        if w is None:
            break
        wazas.append(w)
        print(f"  -> 「{w['name']}」を記録しました（計 {len(wazas)} 技）")

    if not wazas:
        print("何も記録しませんでした。ファイルは変更しません。")
        return 0

    check_silhouettes(wazas)

    if args.replay:
        replay(api, wazas)

    out = Path(args.output)
    if out.exists():
        backup = out.with_suffix(out.suffix + ".bak")
        backup.write_text(out.read_text(encoding="utf-8"), encoding="utf-8")
        print(f"\n既存ファイルを {backup.name} に退避しました。")

    write_atomic(out, {"version": 1, "waza": wazas})
    print(f"{out} に {len(wazas)} 技を書き出しました。")
    for w in wazas:
        n = len(w.get("poses", [1]))
        print(f"  - {w['name']}: {w['setsumei'][:40]}  (ポーズ {n}, 速度 {w['speed']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
