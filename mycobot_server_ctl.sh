#!/usr/bin/env bash
#
# Shell entry point for the MyCobot REST API server control script.
#
# It first makes sure the right Python environment is active — honoring an
# explicit interpreter, a project virtualenv, or pyenv (respecting any
# .python-version in this directory) — and then forwards every argument to
# mycobot_server_ctl.py.
#
# Usage:
#   ./mycobot_server_ctl.sh start|stop|restart|status|run [server options]
#   ./mycobot_server_ctl.sh env          # show which Python is resolved
#
# Environment overrides:
#   PYTHON_BIN   Explicit Python interpreter to use (skips venv/pyenv detection)
#   VENV_DIR     Path to a virtualenv to activate (default: ./.venv then ./venv)
#   PYENV_ROOT   pyenv install root (default: $HOME/.pyenv)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTL="$SCRIPT_DIR/mycobot_server_ctl.py"

# Run from the project directory so pyenv's `.python-version` is picked up.
cd "$SCRIPT_DIR"

log() { printf '%s\n' "$*" >&2; }

# Source a file with `set -u` disabled — activate/init scripts often touch
# unbound variables (PS1, PYTHONHOME, ...).
source_safely() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

resolve_python() {
  # 1. Explicit interpreter wins.
  if [[ -n "${PYTHON_BIN:-}" ]]; then
    log "Using PYTHON_BIN=$PYTHON_BIN"
    return
  fi

  # 2. Project virtualenv.
  local candidate
  for candidate in "${VENV_DIR:-}" "$SCRIPT_DIR/.venv" "$SCRIPT_DIR/venv"; do
    [[ -n "$candidate" ]] || continue
    if [[ -f "$candidate/bin/activate" ]]; then
      log "Activating virtualenv: $candidate"
      source_safely "$candidate/bin/activate"
      PYTHON_BIN="python"
      return
    fi
  done

  # 3. pyenv (respects .python-version in this directory).
  export PYENV_ROOT="${PYENV_ROOT:-$HOME/.pyenv}"
  if [[ -d "$PYENV_ROOT/bin" ]]; then
    export PATH="$PYENV_ROOT/bin:$PATH"
  fi
  if command -v pyenv >/dev/null 2>&1; then
    log "Initializing pyenv (PYENV_ROOT=$PYENV_ROOT)"
    set +u
    eval "$(pyenv init - 2>/dev/null)" || true
    if pyenv commands 2>/dev/null | grep -qx virtualenv-init; then
      eval "$(pyenv virtualenv-init - 2>/dev/null)" || true
    fi
    set -u
    PYTHON_BIN="python"
    return
  fi

  # 4. System fallback.
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
  elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
  else
    log "ERROR: no Python interpreter found."
    log "Set PYTHON_BIN, create a .venv, or install pyenv."
    exit 1
  fi
  log "Using system Python: $PYTHON_BIN"
}

resolve_python

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1 && [[ ! -x "$PYTHON_BIN" ]]; then
  log "ERROR: resolved Python '$PYTHON_BIN' is not executable."
  exit 1
fi

# Wrapper-only diagnostic subcommand.
if [[ "${1:-}" == "env" ]]; then
  log "Resolved Python: $(command -v "$PYTHON_BIN" || echo "$PYTHON_BIN")"
  "$PYTHON_BIN" --version
  exit 0
fi

# Hand off to the Python control script. `exec` keeps signals (Ctrl-C for
# `run`) flowing straight to the controller process.
exec "$PYTHON_BIN" "$CTL" "$@"
