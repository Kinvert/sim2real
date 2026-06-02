#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"
source "$LINE_FOLLOW_ROOT/scripts/line-follow-h4l1-selected.sh"

PORT="${1:-${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}}"
line_follow_h4l1_resolve_checkpoint
STAMP="${LINE_FOLLOW_TIMING_STAMP:-$(date +%Y%m%d-%H%M%S)}"
LOG_FILE="${LINE_FOLLOW_TIMING_LOG:-$LINE_FOLLOW_ROOT/build/line-follow/drive-timing-h4l1-${RUN_ID}-${STAMP}.txt}"
SUMMARY_FILE="${LINE_FOLLOW_TIMING_SUMMARY:-${LOG_FILE%.txt}.summary.txt}"
DRY_RUN="${LINE_FOLLOW_TIMING_DRY_RUN:-0}"

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "Missing H4/L1 checkpoint: $CHECKPOINT" >&2
  exit 1
fi

export LINE_FOLLOW_WEIGHTS="$CHECKPOINT"
export LINE_FOLLOW_HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-4}"
export LINE_FOLLOW_NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-1}"
export LINE_FOLLOW_RUN_MS="${LINE_FOLLOW_RUN_MS:-12000}"
export LINE_FOLLOW_START_DELAY_MS="${LINE_FOLLOW_START_DELAY_MS:-3000}"
export LINE_FOLLOW_PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-20}"
export LINE_FOLLOW_MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
export LINE_FOLLOW_QTI_PIN_PROFILE="${LINE_FOLLOW_QTI_PIN_PROFILE:-p765}"

"$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-live.sh" \
  --drive \
  --pin-profile "$LINE_FOLLOW_QTI_PIN_PROFILE"

ELF="$LINE_FOLLOW_ROOT/build/line-follow/line_follow_model_live_drive.elf"
elf_sha="$(sha256sum "$ELF" | awk '{print $1}')"

mkdir -p "$(dirname "$LOG_FILE")"

echo "Timing run will load to RAM only; EEPROM is not written."
echo "Run ID: $RUN_ID"
echo "Checkpoint: $LINE_FOLLOW_WEIGHTS"
echo "ELF: $ELF"
echo "ELF sha256: $elf_sha"
echo "Port: $PORT"
echo "Log: $LOG_FILE"
echo "Summary: $SUMMARY_FILE"
echo "Start delay: ${LINE_FOLLOW_START_DELAY_MS} ms"
echo "Run cap: ${LINE_FOLLOW_RUN_MS} ms"

if [[ "$DRY_RUN" == "1" ]]; then
  echo "LINE_FOLLOW_TIMING_DRY_RUN=1, skipping RAM load and drive run."
  exit 0
fi

set +e
timeout "${LINE_FOLLOW_LOAD_TIMEOUT:-40s}" \
  "$LINE_FOLLOW_ROOT/scripts/parallax-run-line-follow-model-live.sh" \
    --drive \
    --pin-profile "$LINE_FOLLOW_QTI_PIN_PROFILE" \
    --host-timestamps \
    --log "$LOG_FILE" \
    "$PORT"
status=$?
set -e

if [[ "$status" != "0" && "$status" != "124" ]]; then
  echo "Timing run failed with status $status" >&2
  exit "$status"
fi

"$LINE_FOLLOW_ROOT/scripts/analyze-line-follow-live-log.py" "$LOG_FILE" | tee "$SUMMARY_FILE"
