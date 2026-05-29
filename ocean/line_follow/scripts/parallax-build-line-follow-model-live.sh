#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

ENABLE_DRIVE=0
OUT_NAME="line_follow_model_live"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drive)
      ENABLE_DRIVE=1
      OUT_NAME="line_follow_model_live_drive"
      shift
      ;;
    *)
      echo "Usage: $0 [--drive]" >&2
      exit 2
      ;;
  esac
done

if [[ "$ENABLE_DRIVE" == "1" ]]; then
  MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
else
  MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
fi
LOOP_MS="${LINE_FOLLOW_LOOP_MS:-$LINE_FOLLOW_LOOP_MS_DEFAULT}"

OUT_DIR="$LINE_FOLLOW_ROOT/build/line-follow"
SRC="$LINE_FOLLOW_ROOT/firmware/line_follow_model_live.c"
HEADER="$OUT_DIR/line_follow_model_weights.h"
OUT="$OUT_DIR/$OUT_NAME.elf"

weights="${LINE_FOLLOW_WEIGHTS:-}"
if [[ -z "$weights" ]]; then
  weights="$(find "$SIM2REAL_ROOT/checkpoints/line_follow" -type f -name '*.bin' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-)"
fi

if [[ -z "$weights" || ! -f "$weights" ]]; then
  echo "No line_follow checkpoint found. Set LINE_FOLLOW_WEIGHTS=/path/to/checkpoint.bin" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

"$SIM2REAL_ROOT/.venv/bin/python" \
  "$LINE_FOLLOW_ROOT/scripts/generate-line-follow-model-header.py" \
  "$weights" \
  "$HEADER"

args=(
  -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99
  -DLINE_FOLLOW_ENABLE_DRIVE="$ENABLE_DRIVE"
  -DLINE_FOLLOW_MAX_LOOPS="$MAX_LOOPS"
  -DLINE_FOLLOW_LOOP_MS="$LOOP_MS"
  -DLINE_FOLLOW_PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-1}"
  -DHIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-$LINE_FOLLOW_HIDDEN_SIZE_DEFAULT}"
  -DNUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-$LINE_FOLLOW_NUM_LAYERS_DEFAULT}"
  -DMAX_WHEEL_SPEED_MPS="${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT}"
  -DCOMMAND_DEADBAND="${LINE_FOLLOW_COMMAND_DEADBAND:-$LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT}"
  -DMIN_DRIVE_TICKS_PER_SEC="${LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC:-$LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT}"
  -DQTI_LEFT_PIN="${QTI_LEFT_PIN:-7}"
  -DQTI_MIDDLE_PIN="${QTI_MIDDLE_PIN:-6}"
  -DQTI_RIGHT_PIN="${QTI_RIGHT_PIN:-5}"
  -DQTI_WHITE_TIME="${QTI_WHITE_TIME:-$LINE_FOLLOW_QTI_WHITE_TIME_DEFAULT}"
  -DQTI_BLACK_TIME="${QTI_BLACK_TIME:-$LINE_FOLLOW_QTI_BLACK_TIME_DEFAULT}"
  -DQTI_THRESHOLD_Q1000="${QTI_THRESHOLD_Q1000:-500}"
  -I"$OUT_DIR"
  -I"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools"
  -I"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext"
  -I"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c"
  -L"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools/cmm"
  -L"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext/cmm"
  -L"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c/cmm"
)

libs=(-lsimpletools -lsimpletext -lsimplei2c -lm)

if [[ "$ENABLE_DRIVE" == "1" ]]; then
  args+=(
    -I"$PARALLAX_SIMPLE_LIBS/Robotics/ActivityBot/libabdrive"
    -I"$PARALLAX_SIMPLE_LIBS/TextDevices/libfdserial"
    -L"$PARALLAX_SIMPLE_LIBS/Robotics/ActivityBot/libabdrive/cmm"
    -L"$PARALLAX_SIMPLE_LIBS/TextDevices/libfdserial/cmm"
  )
  libs=(-labdrive -lfdserial "${libs[@]}")
fi

propeller-elf-gcc "${args[@]}" -o "$OUT" "$SRC" "${libs[@]}"

propeller-elf-size "$OUT"
echo "Built $OUT"
echo "Embedded checkpoint: $weights"
echo "Header: $HEADER"
echo "Loop pause: ${LOOP_MS} ms"
echo "Print every: ${LINE_FOLLOW_PRINT_EVERY:-1} loop(s)"
echo "Hidden size: ${LINE_FOLLOW_HIDDEN_SIZE:-$LINE_FOLLOW_HIDDEN_SIZE_DEFAULT}"
echo "Num recurrent layers: ${LINE_FOLLOW_NUM_LAYERS:-$LINE_FOLLOW_NUM_LAYERS_DEFAULT}"
echo "Max wheel speed: ${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT} m/s"
echo "Command deadband: ${LINE_FOLLOW_COMMAND_DEADBAND:-$LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT}"
echo "Minimum drive ticks/sec: ${LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC:-$LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT}"
if [[ "$ENABLE_DRIVE" == "1" ]]; then
  if [[ "$MAX_LOOPS" == "0" ]]; then
    echo "Drive output is ENABLED with no loop cap. Use only with the robot safely staged."
  else
    echo "Drive output is ENABLED for $MAX_LOOPS loops. Use only with the robot safely staged."
  fi
else
  echo "Drive output is disabled. This build prints live QTI/model telemetry only."
fi
