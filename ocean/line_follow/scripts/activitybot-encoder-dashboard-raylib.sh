#!/usr/bin/env bash
set -euo pipefail

LINE_FOLLOW_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM2REAL_ROOT="$(git -C "$LINE_FOLLOW_ROOT" rev-parse --show-toplevel)"
PORT="${1:-/dev/ttyUSB0}"
DISPLAY_VALUE="${DISPLAY:-:0}"
PY="$SIM2REAL_ROOT/.venv/bin/python"
APP="$LINE_FOLLOW_ROOT/scripts/activitybot-encoder-dashboard-raylib.py"

CMD="cd '$LINE_FOLLOW_ROOT' && DISPLAY='$DISPLAY_VALUE' '$PY' '$APP' --port '$PORT'"

if id -nG | grep -qw dialout; then
  exec bash -lc "$CMD"
else
  exec sg dialout -c "$CMD"
fi
