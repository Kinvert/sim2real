#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"
source "$LINE_FOLLOW_ROOT/scripts/line-follow-h4l1-selected.sh"

PORT="${1:-${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}}"
line_follow_h4l1_resolve_checkpoint
OUT="$LINE_FOLLOW_ROOT/build/line-follow/line_follow_model_live_drive_p765_h4l1_${RUN_ID}_deploy30s_start8s_quiet.elf"

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "Missing H4/L1 checkpoint: $CHECKPOINT" >&2
  exit 1
fi

export LINE_FOLLOW_WEIGHTS="$CHECKPOINT"
export LINE_FOLLOW_HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-4}"
export LINE_FOLLOW_NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-1}"
export LINE_FOLLOW_RUN_MS="${LINE_FOLLOW_RUN_MS:-30000}"
export LINE_FOLLOW_START_DELAY_MS="${LINE_FOLLOW_START_DELAY_MS:-8000}"
export LINE_FOLLOW_STARTUP_PRINTS="${LINE_FOLLOW_STARTUP_PRINTS:-0}"
export LINE_FOLLOW_PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-0}"
export LINE_FOLLOW_MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
export LINE_FOLLOW_QTI_PIN_PROFILE="${LINE_FOLLOW_QTI_PIN_PROFILE:-p765}"

"$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-live.sh" \
  --drive \
  --pin-profile "$LINE_FOLLOW_QTI_PIN_PROFILE"

cp "$LINE_FOLLOW_ROOT/build/line-follow/line_follow_model_live_drive.elf" "$OUT"

echo "Flashing p765 H4/L1 start-delay drive image to EEPROM."
echo "Run ID: $RUN_ID"
echo "Checkpoint: $LINE_FOLLOW_WEIGHTS"
echo "ELF: $OUT"
echo "Port: $PORT"
echo "This uses propeller-load -e without -r/-t, so it does not intentionally start the drive program."

exec propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -e "$OUT"
