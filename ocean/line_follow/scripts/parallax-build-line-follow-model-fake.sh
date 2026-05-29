#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

OUT_DIR="$LINE_FOLLOW_ROOT/build/line-follow"
SRC="$LINE_FOLLOW_ROOT/firmware/line_follow_model_fake.c"
HEADER="$OUT_DIR/line_follow_model_weights.h"
OUT="$OUT_DIR/line_follow_model_fake.elf"

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

propeller-elf-gcc \
  -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 \
  -DHIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-$LINE_FOLLOW_HIDDEN_SIZE_DEFAULT}" \
  -DNUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-$LINE_FOLLOW_NUM_LAYERS_DEFAULT}" \
  -DMAX_WHEEL_SPEED_MPS="${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT}" \
  -DCOMMAND_DEADBAND="${LINE_FOLLOW_COMMAND_DEADBAND:-$LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT}" \
  -DMIN_DRIVE_TICKS_PER_SEC="${LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC:-$LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT}" \
  -I"$OUT_DIR" \
  -I"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools" \
  -I"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext" \
  -I"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c" \
  -L"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools/cmm" \
  -L"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext/cmm" \
  -L"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c/cmm" \
  -o "$OUT" "$SRC" \
  -lsimpletools -lsimpletext -lsimplei2c -lm

propeller-elf-size "$OUT"
echo "Built $OUT"
echo "Embedded checkpoint: $weights"
echo "Header: $HEADER"
echo "Hidden size: ${LINE_FOLLOW_HIDDEN_SIZE:-$LINE_FOLLOW_HIDDEN_SIZE_DEFAULT}"
echo "Num recurrent layers: ${LINE_FOLLOW_NUM_LAYERS:-$LINE_FOLLOW_NUM_LAYERS_DEFAULT}"
echo "Max wheel speed: ${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT} m/s"
echo "Command deadband: ${LINE_FOLLOW_COMMAND_DEADBAND:-$LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT}"
echo "Minimum drive ticks/sec: ${LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC:-$LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT}"
echo "This firmware runs trained-model inference on fake observations. It never reads QTI pins or drives motors."
