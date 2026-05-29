#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

OUT_DIR="$LINE_FOLLOW_ROOT/build/line-follow"
SRC="$LINE_FOLLOW_ROOT/firmware/line_follow_sd_dump.c"
OUT="$OUT_DIR/line_follow_sd_dump.elf"
SD_FILE="${LINE_FOLLOW_SD_FILE:-lf_log.bin}"

mkdir -p "$OUT_DIR"

propeller-elf-gcc \
  -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99 \
  -DLINE_FOLLOW_SD_FILE="\"$SD_FILE\"" \
  -DLINE_FOLLOW_SD_DO_PIN="${LINE_FOLLOW_SD_DO_PIN:-22}" \
  -DLINE_FOLLOW_SD_CLK_PIN="${LINE_FOLLOW_SD_CLK_PIN:-23}" \
  -DLINE_FOLLOW_SD_DI_PIN="${LINE_FOLLOW_SD_DI_PIN:-24}" \
  -DLINE_FOLLOW_SD_CS_PIN="${LINE_FOLLOW_SD_CS_PIN:-25}" \
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
echo "SD file: $SD_FILE"
