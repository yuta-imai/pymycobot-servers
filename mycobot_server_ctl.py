#!/usr/bin/env python3
"""Atomic start/stop control for the MyCobot REST API server.

The REST API server (`mycobot_api_server.py`) owns the robot controller and its
serial connection: it opens the connection on startup and releases it on a
graceful shutdown. This wrapper supervises that single process as one atomic
unit so that:

  * startup is health-gated — `start` only succeeds once the server actually
    answers on `/health`; otherwise the child is torn down and nothing is left
    half-running;
  * shutdown is guaranteed — `stop` signals the whole process group, gives the
    server time to run its graceful shutdown (returning the serial port), then
    force-kills any stragglers, so there are never orphaned uvicorn workers or
    a locked serial port.

Commands: start | stop | restart | status | run (foreground).
Only the Python standard library is used.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
SERVER_SCRIPT = HERE / "mycobot_api_server.py"

DEFAULT_PID_FILE = HERE / ".mycobot_api_server.pid"
DEFAULT_LOG_FILE = HERE / "mycobot_api_server.log"
DEFAULT_STARTUP_TIMEOUT = 30.0
DEFAULT_STOP_TIMEOUT = 15.0


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def health_url(host: str, port: int) -> str:
    # 0.0.0.0 / :: are bind addresses, not connectable; poll loopback instead.
    if host in ("0.0.0.0", "::", ""):
        host = "127.0.0.1"
    return f"http://{host}:{port}/health"


def probe_health(url: str, timeout: float = 1.0) -> dict | None:
    """Return the parsed /health JSON if reachable, else None."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            if resp.status != 200:
                return None
            import json

            return json.loads(resp.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError):
        return None


def read_pid(pid_file: Path) -> int | None:
    try:
        return int(pid_file.read_text().strip())
    except (FileNotFoundError, ValueError):
        return None


def process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def running_pid(pid_file: Path) -> int | None:
    """Return the PID from the file only if that process is still alive."""
    pid = read_pid(pid_file)
    if pid is not None and process_alive(pid):
        return pid
    return None


def build_server_cmd(args: argparse.Namespace) -> list[str]:
    cmd = [
        sys.executable,
        str(SERVER_SCRIPT),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--robot-port",
        args.robot_port,
        "--robot-baudrate",
        str(args.robot_baudrate),
    ]
    if args.reload:
        cmd.append("--reload")
    return cmd


def terminate_group(pid: int, stop_timeout: float) -> None:
    """Gracefully stop the process group, escalating to SIGKILL on timeout."""
    try:
        pgid = os.getpgid(pid)
    except ProcessLookupError:
        return

    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return

    deadline = time.monotonic() + stop_timeout
    while time.monotonic() < deadline:
        if not process_alive(pid):
            return
        time.sleep(0.2)

    # Graceful window elapsed — force kill whatever remains.
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


# --------------------------------------------------------------------------- #
# Commands
# --------------------------------------------------------------------------- #
def cmd_start(args: argparse.Namespace, *, foreground: bool = False) -> int:
    pid_file = Path(args.pid_file)

    existing = running_pid(pid_file)
    if existing is not None:
        print(f"Already running (pid {existing}).")
        return 0
    # Stale pid file, if any.
    if pid_file.exists():
        pid_file.unlink()

    url = health_url(args.host, args.port)
    cmd = build_server_cmd(args)

    if foreground:
        log_target = None  # inherit our stdout/stderr
        print(f"Starting API server (foreground): {' '.join(cmd)}")
    else:
        log_file = Path(args.log_file)
        log_target = open(log_file, "ab")
        print(f"Starting API server: {' '.join(cmd)}")
        print(f"Logging to {log_file}")

    # start_new_session=True puts the child in its own process group/session so
    # we can signal the whole tree (uvicorn + reload workers) and so terminal
    # Ctrl-C does not race our own graceful-shutdown handling.
    proc = subprocess.Popen(
        cmd,
        cwd=str(HERE),
        stdout=log_target,
        stderr=subprocess.STDOUT if log_target else None,
        start_new_session=True,
    )
    if log_target:
        log_target.close()

    pid_file.write_text(str(proc.pid))

    # Install signal handlers *before* the health-gate so that a Ctrl-C at any
    # point during startup also tears the child down — never an orphan. The
    # handler only flips a flag; the actual teardown happens in the main flow.
    stop_requested = {"flag": False}

    def handle(signum, _frame):
        stop_requested["flag"] = True

    prev_int = signal.signal(signal.SIGINT, handle)
    prev_term = signal.signal(signal.SIGTERM, handle)

    def restore_handlers():
        signal.signal(signal.SIGINT, prev_int)
        signal.signal(signal.SIGTERM, prev_term)

    def abort(message: str) -> int:
        print(message)
        terminate_group(proc.pid, args.stop_timeout)
        pid_file.unlink(missing_ok=True)
        restore_handlers()
        return 1

    # Health-gate: wait until the server actually serves, or fail atomically.
    deadline = time.monotonic() + args.startup_timeout
    health = None
    while time.monotonic() < deadline:
        if stop_requested["flag"]:
            return abort("\nInterrupted during startup; stopping server.")
        if proc.poll() is not None:
            print(f"Server process exited during startup (code {proc.returncode}).")
            pid_file.unlink(missing_ok=True)
            restore_handlers()
            return 1
        health = probe_health(url)
        if health is not None:
            break
        time.sleep(0.25)

    if health is None:
        return abort(
            f"Server did not become healthy within {args.startup_timeout:.0f}s; stopping."
        )

    robot = health.get("robot_connected")
    robot_note = "robot connected" if robot else "robot NOT connected (degraded)"
    print(f"API server is up (pid {proc.pid}, {url}) — {robot_note}.")

    if not foreground:
        # Background start: leave the server running independently.
        restore_handlers()
        return 0

    # Foreground supervisor: block until the child exits or a signal arrives,
    # then tear the whole group down gracefully.
    print("Press Ctrl-C to stop.")
    while proc.poll() is None and not stop_requested["flag"]:
        time.sleep(0.2)

    if stop_requested["flag"]:
        print("\nShutting down server...")
        terminate_group(proc.pid, args.stop_timeout)

    proc.wait()
    pid_file.unlink(missing_ok=True)
    restore_handlers()
    print("Server stopped.")
    return 0


def cmd_stop(args: argparse.Namespace) -> int:
    pid_file = Path(args.pid_file)
    pid = running_pid(pid_file)
    if pid is None:
        if pid_file.exists():
            pid_file.unlink()
            print("Not running (removed stale pid file).")
        else:
            print("Not running.")
        return 0

    print(f"Stopping API server (pid {pid})...")
    terminate_group(pid, args.stop_timeout)
    pid_file.unlink(missing_ok=True)

    if process_alive(pid):
        print(f"Failed to stop pid {pid}.")
        return 1
    print("Server stopped.")
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    pid_file = Path(args.pid_file)
    pid = running_pid(pid_file)
    if pid is None:
        print("Status: stopped")
        return 3  # LSB convention: program not running

    health = probe_health(health_url(args.host, args.port))
    if health is None:
        print(f"Status: running (pid {pid}) but not answering /health")
        return 0
    robot = health.get("robot_connected")
    print(
        f"Status: running (pid {pid}), health={health.get('status')}, "
        f"robot_connected={robot}"
    )
    return 0


def cmd_restart(args: argparse.Namespace) -> int:
    cmd_stop(args)
    return cmd_start(args)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Atomic start/stop control for the MyCobot REST API server."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    def add_common(p: argparse.ArgumentParser) -> None:
        # Server connection options (mirror mycobot_api_server.py).
        p.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
        p.add_argument("--port", type=int, default=8080, help="Bind port (default: 8080)")
        p.add_argument(
            "--robot-port", default="/dev/ttyACM0", help="Robot serial port"
        )
        p.add_argument(
            "--robot-baudrate", type=int, default=115200, help="Robot serial baudrate"
        )
        p.add_argument("--reload", action="store_true", help="Enable auto-reload (dev)")
        # Wrapper options.
        p.add_argument("--pid-file", default=str(DEFAULT_PID_FILE))
        p.add_argument("--log-file", default=str(DEFAULT_LOG_FILE))
        p.add_argument(
            "--startup-timeout", type=float, default=DEFAULT_STARTUP_TIMEOUT,
            help="Seconds to wait for /health during start",
        )
        p.add_argument(
            "--stop-timeout", type=float, default=DEFAULT_STOP_TIMEOUT,
            help="Seconds to wait for graceful shutdown before SIGKILL",
        )

    for name, help_text in [
        ("start", "Start the server in the background (health-gated)"),
        ("stop", "Stop the server gracefully"),
        ("restart", "Stop then start"),
        ("status", "Show whether the server is running and healthy"),
        ("run", "Run the server in the foreground (Ctrl-C stops it cleanly)"),
    ]:
        p = sub.add_parser(name, help=help_text)
        add_common(p)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "start":
        return cmd_start(args)
    if args.command == "run":
        return cmd_start(args, foreground=True)
    if args.command == "stop":
        return cmd_stop(args)
    if args.command == "restart":
        return cmd_restart(args)
    if args.command == "status":
        return cmd_status(args)
    return 2


if __name__ == "__main__":
    sys.exit(main())
