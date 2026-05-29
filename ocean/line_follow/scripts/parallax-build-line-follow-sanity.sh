#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

ENABLE_DRIVE=0
FAKE_OBS=0
OUT_NAME="line_follow_sanity"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fake)
      FAKE_OBS=1
      OUT_NAME="line_follow_sanity_fake"
      shift
      ;;
    --drive)
      ENABLE_DRIVE=1
      OUT_NAME="line_follow_sanity_drive"
      shift
      ;;
    *)
      echo "Usage: $0 [--fake|--drive]" >&2
      exit 2
      ;;
  esac
done

if [[ "$FAKE_OBS" == "1" && "$ENABLE_DRIVE" == "1" ]]; then
  echo "--fake and --drive are intentionally mutually exclusive" >&2
  exit 2
fi

if [[ "$ENABLE_DRIVE" == "1" ]]; then
  MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-400}"
else
  MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
fi
LOOP_MS="${LINE_FOLLOW_LOOP_MS:-$LINE_FOLLOW_LOOP_MS_DEFAULT}"

OUT_DIR="$LINE_FOLLOW_ROOT/build/line-follow"
SRC="$LINE_FOLLOW_ROOT/firmware/line_follow_sanity.c"
OUT="$OUT_DIR/$OUT_NAME.elf"

mkdir -p "$OUT_DIR"

args=(
  -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99
  -DLINE_FOLLOW_ENABLE_DRIVE="$ENABLE_DRIVE"
  -DLINE_FOLLOW_FAKE_OBS="$FAKE_OBS"
  -DLINE_FOLLOW_MAX_LOOPS="$MAX_LOOPS"
  -DQTI_LEFT_PIN="${QTI_LEFT_PIN:-7}"
  -DQTI_MIDDLE_PIN="${QTI_MIDDLE_PIN:-6}"
  -DQTI_RIGHT_PIN="${QTI_RIGHT_PIN:-5}"
  -DQTI_WHITE_TIME="${QTI_WHITE_TIME:-$LINE_FOLLOW_QTI_WHITE_TIME_DEFAULT}"
  -DQTI_BLACK_TIME="${QTI_BLACK_TIME:-$LINE_FOLLOW_QTI_BLACK_TIME_DEFAULT}"
  -DQTI_THRESHOLD_Q1000="${QTI_THRESHOLD_Q1000:-500}"
  -DLINE_FOLLOW_BASE_TICKS="${LINE_FOLLOW_BASE_TICKS:-20}"
  -DLINE_FOLLOW_TURN_TICKS="${LINE_FOLLOW_TURN_TICKS:-18}"
  -DLINE_FOLLOW_SEARCH_TICKS="${LINE_FOLLOW_SEARCH_TICKS:-12}"
  -DLINE_FOLLOW_MAX_TICKS="${LINE_FOLLOW_MAX_TICKS:-40}"
  -DLINE_FOLLOW_LOOP_MS="$LOOP_MS"
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
echo "Loop pause: ${LOOP_MS} ms"
if [[ "$ENABLE_DRIVE" == "1" ]]; then
  echo "Drive output is ENABLED for $MAX_LOOPS loops. Use only with the robot safely staged."
elif [[ "$FAKE_OBS" == "1" ]]; then
  echo "Fake observation mode. This build prints canned obs->command cases and never drives motors."
else
  echo "Drive output is disabled. This build only streams QTI and policy-stub telemetry."
fi
