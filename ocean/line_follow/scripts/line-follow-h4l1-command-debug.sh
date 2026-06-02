#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"
source "$LINE_FOLLOW_ROOT/scripts/line-follow-h4l1-selected.sh"

PORT="${1:-${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}}"
line_follow_h4l1_resolve_checkpoint
STAMP="${LINE_FOLLOW_DEBUG_STAMP:-$(date +%Y%m%d-%H%M%S)}"
LOG_FILE="${LINE_FOLLOW_DEBUG_LOG:-$LINE_FOLLOW_ROOT/build/line-follow/command-debug-h4l1-${RUN_ID}-${STAMP}.txt}"
SUMMARY_FILE="${LINE_FOLLOW_DEBUG_SUMMARY:-${LOG_FILE%.txt}.summary.txt}"

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "Missing H4/L1 checkpoint: $CHECKPOINT" >&2
  exit 1
fi

export LINE_FOLLOW_WEIGHTS="$CHECKPOINT"
export LINE_FOLLOW_HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-4}"
export LINE_FOLLOW_NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-1}"
export LINE_FOLLOW_PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-1}"
export LINE_FOLLOW_MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-80}"
export LINE_FOLLOW_RUN_MS="${LINE_FOLLOW_RUN_MS:-0}"
export LINE_FOLLOW_START_DELAY_MS="${LINE_FOLLOW_START_DELAY_MS:-0}"
export LINE_FOLLOW_QTI_PIN_PROFILE="${LINE_FOLLOW_QTI_PIN_PROFILE:-p765}"

"$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-live.sh" \
  --pin-profile "$LINE_FOLLOW_QTI_PIN_PROFILE"

mkdir -p "$(dirname "$LOG_FILE")"

echo "Command debug loads to RAM only; EEPROM is not written and drive is disabled."
echo "Run ID: $RUN_ID"
echo "Checkpoint: $LINE_FOLLOW_WEIGHTS"
echo "Port: $PORT"
echo "Log: $LOG_FILE"
echo "Summary: $SUMMARY_FILE"

set +e
timeout "${LINE_FOLLOW_LOAD_TIMEOUT:-25s}" \
  "$LINE_FOLLOW_ROOT/scripts/parallax-run-line-follow-model-live.sh" \
    --pin-profile "$LINE_FOLLOW_QTI_PIN_PROFILE" \
    --host-timestamps \
    --log "$LOG_FILE" \
    "$PORT"
status=$?
set -e

if [[ "$status" != "0" && "$status" != "124" ]]; then
  echo "Command debug run failed with status $status" >&2
  exit "$status"
fi

"$LINE_FOLLOW_ROOT/scripts/analyze-line-follow-live-log.py" "$LOG_FILE" | tee "$SUMMARY_FILE"
