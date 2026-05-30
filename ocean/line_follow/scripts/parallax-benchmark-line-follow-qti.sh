#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

PORT="${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}"
LOOPS="${LINE_FOLLOW_MAX_LOOPS:-300}"
PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-50}"
TIMEOUT_SECONDS="${LINE_FOLLOW_QTI_BENCH_TIMEOUT_SECONDS:-45}"
LOG_DIR="$LINE_FOLLOW_ROOT/build/line-follow"

while [[ $# -gt 0 ]]; do
  case "$1" in
    /dev/*)
      PORT="$1"
      shift
      ;;
    *)
      echo "Usage: $0 [/dev/ttyUSB0]" >&2
      exit 2
      ;;
  esac
done

mkdir -p "$LOG_DIR"

for mode in 0 1 2; do
  case "$mode" in
    0) label="sequential" ;;
    1) label="grouped_c" ;;
    2) label="pasm_cog" ;;
  esac
  log="$LOG_DIR/qti-${label}-last.txt"
  echo
  echo "=== QTI mode $mode ($label): loops=$LOOPS print_every=$PRINT_EVERY ==="
  LINE_FOLLOW_QTI_MODE="$mode" \
    LINE_FOLLOW_MAX_LOOPS="$LOOPS" \
    LINE_FOLLOW_PRINT_EVERY="$PRINT_EVERY" \
    "$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-live.sh"

  status=0
  timeout "${TIMEOUT_SECONDS}s" \
    "$LINE_FOLLOW_ROOT/scripts/parallax-run-line-follow-model-live.sh" \
      --host-timestamps --log "$log" "$PORT" || status=$?
  if [[ "$status" != "0" && "$status" != "124" ]]; then
    exit "$status"
  fi
  "$LINE_FOLLOW_ROOT/scripts/analyze-line-follow-live-log.py" "$log"
done
