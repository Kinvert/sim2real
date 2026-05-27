#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-/dev/ttyUSB0}"
DISPLAY_VALUE="${DISPLAY:-:0}"
PY="$ROOT/.venv/bin/python"
APP="$ROOT/scripts/activitybot-encoder-dashboard-raylib.py"

CMD="cd '$ROOT' && DISPLAY='$DISPLAY_VALUE' '$PY' '$APP' --port '$PORT'"

if id -nG | grep -qw dialout; then
  exec bash -lc "$CMD"
else
  exec sg dialout -c "$CMD"
fi
