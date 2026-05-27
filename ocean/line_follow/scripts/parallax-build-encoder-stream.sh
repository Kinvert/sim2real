#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

OUT_DIR="$LINE_FOLLOW_ROOT/build/parallax-smoke"
SRC="$LINE_FOLLOW_ROOT/firmware/smoke/encoder_stream.c"
OUT="$OUT_DIR/encoder_stream.elf"

mkdir -p "$OUT_DIR"

propeller-elf-gcc \
  -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 \
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
