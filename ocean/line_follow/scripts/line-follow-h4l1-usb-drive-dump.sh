#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"
source "$LINE_FOLLOW_ROOT/scripts/line-follow-h4l1-selected.sh"

PORT="${1:-${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}}"
line_follow_h4l1_resolve_checkpoint
line_follow_h4l1_apply_run_deploy_env

STAMP="${LINE_FOLLOW_USB_DUMP_STAMP:-$(date +%Y%m%d-%H%M%S)}"
RUN_MS="${LINE_FOLLOW_RUN_MS:-1000}"
START_DELAY_MS="${LINE_FOLLOW_START_DELAY_MS:-3000}"
PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-1}"
LOG_FILE="${LINE_FOLLOW_USB_DUMP_LOG:-$LINE_FOLLOW_ROOT/build/line-follow/usb-drive-dump-h4l1-${RUN_ID}-${STAMP}.txt}"
SUMMARY_FILE="${LINE_FOLLOW_USB_DUMP_SUMMARY:-${LOG_FILE%.txt}.summary.txt}"
DRY_RUN="${LINE_FOLLOW_USB_DUMP_DRY_RUN:-0}"

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "Missing H4/L1 checkpoint: $CHECKPOINT" >&2
  exit 1
fi

export LINE_FOLLOW_WEIGHTS="$CHECKPOINT"
export LINE_FOLLOW_HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-4}"
export LINE_FOLLOW_NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-1}"
export LINE_FOLLOW_RUN_MS="$RUN_MS"
export LINE_FOLLOW_START_DELAY_MS="$START_DELAY_MS"
export LINE_FOLLOW_PRINT_EVERY="$PRINT_EVERY"
export LINE_FOLLOW_MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
export LINE_FOLLOW_STARTUP_PRINTS="${LINE_FOLLOW_STARTUP_PRINTS:-0}"
export LINE_FOLLOW_STATUS_LEDS="${LINE_FOLLOW_STATUS_LEDS:-1}"
export LINE_FOLLOW_QTI_PIN_PROFILE="${LINE_FOLLOW_QTI_PIN_PROFILE:-p765}"

"$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-live.sh" \
  --drive \
  --pin-profile "$LINE_FOLLOW_QTI_PIN_PROFILE"

ELF="$LINE_FOLLOW_ROOT/build/line-follow/line_follow_model_live_drive.elf"
elf_sha="$(sha256sum "$ELF" | awk '{print $1}')"
mkdir -p "$(dirname "$LOG_FILE")"

echo "H4/L1 USB drive dump loads to RAM only; EEPROM is not written."
echo "This intentionally commands motors for a short run while tethered."
echo "Run ID: $RUN_ID"
echo "Checkpoint: $LINE_FOLLOW_WEIGHTS"
echo "ELF: $ELF"
echo "ELF sha256: $elf_sha"
echo "Port: $PORT"
echo "Run cap: ${RUN_MS} ms"
echo "Start delay: ${START_DELAY_MS} ms"
echo "Print every: ${PRINT_EVERY} loop(s)"
echo "Log: $LOG_FILE"
echo "Summary: $SUMMARY_FILE"

if [[ "$DRY_RUN" == "1" ]]; then
  echo "LINE_FOLLOW_USB_DUMP_DRY_RUN=1, skipping RAM load and drive run."
  exit 0
fi

set +e
timeout "${LINE_FOLLOW_LOAD_TIMEOUT:-18s}" \
  "$LINE_FOLLOW_ROOT/scripts/parallax-run-line-follow-model-live.sh" \
    --drive \
    --pin-profile "$LINE_FOLLOW_QTI_PIN_PROFILE" \
    --host-timestamps \
    --log "$LOG_FILE" \
    "$PORT"
status=$?
set -e

if [[ "$status" != "0" && "$status" != "124" ]]; then
  echo "USB drive dump failed with status $status" >&2
  exit "$status"
fi

"$LINE_FOLLOW_ROOT/scripts/analyze-line-follow-live-log.py" "$LOG_FILE" \
  | tee "$SUMMARY_FILE"
