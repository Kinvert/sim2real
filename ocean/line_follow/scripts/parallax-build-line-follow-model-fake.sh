#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

OUT_DIR="$LINE_FOLLOW_ROOT/build/line-follow"
SRC="$LINE_FOLLOW_ROOT/firmware/line_follow_model_fake.c"
HEADER="$OUT_DIR/line_follow_model_weights.h"
OUT="$OUT_DIR/line_follow_model_fake.elf"
HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-$LINE_FOLLOW_HIDDEN_SIZE_DEFAULT}"
NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-$LINE_FOLLOW_NUM_LAYERS_DEFAULT}"
MAX_WHEEL_SPEED_MPS="${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT}"
COMMAND_DEADBAND="${LINE_FOLLOW_COMMAND_DEADBAND:-$LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT}"
MIN_DRIVE_TICKS_PER_SEC="${LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC:-$LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT}"
MAX_WHEEL_TICKS_PER_SEC_Q1000="$(
  awk -v mps="$MAX_WHEEL_SPEED_MPS" 'BEGIN {
    printf "%d", (mps / (3.14159265358979323846 * 0.065)) * 64.0 * 1000.0 + 0.5
  }'
)"
COMMAND_DEADBAND_Q1000="$(
  awk -v deadband="$COMMAND_DEADBAND" 'BEGIN { printf "%d", deadband * 1000.0 + 0.5 }'
)"

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
  --hidden-size "$HIDDEN_SIZE" \
  --num-layers "$NUM_LAYERS" \
  "$weights" \
  "$HEADER"

args=(
  -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 \
  -DHIDDEN_SIZE="$HIDDEN_SIZE" \
  -DNUM_LAYERS="$NUM_LAYERS" \
  -DMAX_WHEEL_SPEED_MPS="$MAX_WHEEL_SPEED_MPS" \
  -DCOMMAND_DEADBAND="$COMMAND_DEADBAND" \
  -DCOMMAND_DEADBAND_Q1000="$COMMAND_DEADBAND_Q1000" \
  -DMIN_DRIVE_TICKS_PER_SEC="$MIN_DRIVE_TICKS_PER_SEC" \
  -DMAX_WHEEL_TICKS_PER_SEC_Q1000="$MAX_WHEEL_TICKS_PER_SEC_Q1000" \
  -I"$OUT_DIR" \
  -I"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools" \
  -I"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext" \
  -I"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c" \
  -L"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools/cmm" \
  -L"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext/cmm" \
  -L"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c/cmm"
)

if [[ -n "${LINE_FOLLOW_INFERENCE_MODE:-}" ]]; then
  args+=(-DLINE_FOLLOW_INFERENCE_MODE="$LINE_FOLLOW_INFERENCE_MODE")
fi

propeller-elf-gcc "${args[@]}" -o "$OUT" "$SRC" \
  -lsimpletools -lsimpletext -lsimplei2c -lm

propeller-elf-size "$OUT"
echo "Built $OUT"
echo "Embedded checkpoint: $weights"
echo "Header: $HEADER"
echo "Hidden size: $HIDDEN_SIZE"
echo "Num recurrent layers: $NUM_LAYERS"
echo "Inference mode: ${LINE_FOLLOW_INFERENCE_MODE:-auto}"
echo "Max wheel speed: $MAX_WHEEL_SPEED_MPS m/s"
echo "Command deadband: $COMMAND_DEADBAND"
echo "Minimum drive ticks/sec: $MIN_DRIVE_TICKS_PER_SEC"
echo "This firmware runs trained-model inference on fake observations. It never reads QTI pins or drives motors."
