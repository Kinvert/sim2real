#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"
source "$LINE_FOLLOW_ROOT/scripts/line-follow-h4l1-selected.sh"

PORT="${1:-${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}}"
line_follow_h4l1_resolve_checkpoint
line_follow_h4l1_apply_run_deploy_env
PIN_PROFILE="${LINE_FOLLOW_QTI_PIN_PROFILE:-p765}"
RUN_MS="${LINE_FOLLOW_RUN_MS:-30000}"
START_DELAY_MS="${LINE_FOLLOW_START_DELAY_MS:-2000}"
STATUS_LEDS="${LINE_FOLLOW_STATUS_LEDS:-1}"
STARTUP_PRINTS="${LINE_FOLLOW_STARTUP_PRINTS:-0}"
PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-0}"
MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
DEPLOY_TAG="${LINE_FOLLOW_DEPLOY_TAG:-deploy}"
FLASH="${LINE_FOLLOW_FLASH:-1}"
FLASH_LOG="${LINE_FOLLOW_FLASH_LOG:-}"

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "Missing H4/L1 checkpoint: $CHECKPOINT" >&2
  exit 1
fi

led_suffix="quiet"
if [[ "$STATUS_LEDS" != "0" ]]; then
  led_suffix="statusleds"
fi

build_id="${LINE_FOLLOW_BUILD_ID:-${RUN_ID: -8}}"
out="$LINE_FOLLOW_ROOT/build/line-follow/line_follow_model_live_drive_${PIN_PROFILE}_h4l1_${RUN_ID}_${DEPLOY_TAG}_${RUN_MS}ms_start${START_DELAY_MS}ms_${led_suffix}.elf"

export LINE_FOLLOW_WEIGHTS="$CHECKPOINT"
export LINE_FOLLOW_HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-4}"
export LINE_FOLLOW_NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-1}"
export LINE_FOLLOW_RUN_MS="$RUN_MS"
export LINE_FOLLOW_START_DELAY_MS="$START_DELAY_MS"
export LINE_FOLLOW_STARTUP_PRINTS="$STARTUP_PRINTS"
export LINE_FOLLOW_STATUS_LEDS="$STATUS_LEDS"
export LINE_FOLLOW_PRINT_EVERY="$PRINT_EVERY"
export LINE_FOLLOW_MAX_LOOPS="$MAX_LOOPS"
export LINE_FOLLOW_QTI_PIN_PROFILE="$PIN_PROFILE"
export LINE_FOLLOW_BUILD_ID="$build_id"

"$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-live.sh" \
  --drive \
  --pin-profile "$PIN_PROFILE"

cp "$LINE_FOLLOW_ROOT/build/line-follow/line_follow_model_live_drive.elf" "$out"
elf_sha="$(sha256sum "$out" | awk '{print $1}')"

echo "Prepared ${PIN_PROFILE} H4/L1 deploy image."
echo "Run ID: $RUN_ID"
echo "Checkpoint: $LINE_FOLLOW_WEIGHTS"
echo "ELF: $out"
echo "ELF sha256: $elf_sha"
echo "Port: $PORT"
echo "Run cap: ${RUN_MS} ms"
echo "Start delay: ${START_DELAY_MS} ms"
echo "Status LEDs: $STATUS_LEDS"
echo "After EEPROM flash, power-cycle or reset the Propeller before the motor test."
echo "Changing ActivityBot power from 1 to 2 only enables motors; it may not restart the EEPROM program."

if [[ "$FLASH" == "0" ]]; then
  echo "LINE_FOLLOW_FLASH=0, skipping EEPROM write."
  exit 0
fi

echo "Flashing to EEPROM with propeller-load -e only; this does not intentionally start the drive program."
if [[ -n "$FLASH_LOG" ]]; then
  mkdir -p "$(dirname "$FLASH_LOG")"
  echo "Flash log: $FLASH_LOG"
  propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -e "$out" 2>&1 | tee "$FLASH_LOG"
  exit "${PIPESTATUS[0]}"
fi

exec propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -e "$out"
