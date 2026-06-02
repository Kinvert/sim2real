#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

OUT_DIR="$LINE_FOLLOW_ROOT/build/parallax-smoke"
SRC="$LINE_FOLLOW_ROOT/firmware/smoke/drive_ticks_probe.c"
OUT="$OUT_DIR/drive_ticks_probe.elf"

mkdir -p "$OUT_DIR"

propeller-elf-gcc \
  -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 \
  -DDRIVE_LEFT_TICKS="${DRIVE_LEFT_TICKS:-2}" \
  -DDRIVE_RIGHT_TICKS="${DRIVE_RIGHT_TICKS:-2}" \
  -DDRIVE_RUN_MS="${DRIVE_RUN_MS:-3000}" \
  -DDRIVE_PRINT_MS="${DRIVE_PRINT_MS:-250}" \
  -I"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools" \
  -I"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext" \
  -I"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c" \
  -I"$PARALLAX_SIMPLE_LIBS/Robotics/ActivityBot/libabdrive" \
  -I"$PARALLAX_SIMPLE_LIBS/TextDevices/libfdserial" \
  -L"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools/cmm" \
  -L"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext/cmm" \
  -L"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c/cmm" \
  -L"$PARALLAX_SIMPLE_LIBS/Robotics/ActivityBot/libabdrive/cmm" \
  -L"$PARALLAX_SIMPLE_LIBS/TextDevices/libfdserial/cmm" \
  -o "$OUT" "$SRC" \
  -labdrive -lfdserial -lsimpletools -lsimpletext -lsimplei2c -lm

propeller-elf-size "$OUT"
echo "Built $OUT"
echo "Command: left=${DRIVE_LEFT_TICKS:-2} right=${DRIVE_RIGHT_TICKS:-2} run_ms=${DRIVE_RUN_MS:-3000} print_ms=${DRIVE_PRINT_MS:-250}"
