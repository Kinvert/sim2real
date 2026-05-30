#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

ENABLE_DRIVE=0
SD_LOG=0
OUT_NAME="line_follow_model_live"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drive)
      ENABLE_DRIVE=1
      shift
      ;;
    --sd-log)
      SD_LOG=1
      shift
      ;;
    *)
      echo "Usage: $0 [--drive] [--sd-log]" >&2
      exit 2
      ;;
  esac
done

if [[ "$ENABLE_DRIVE" == "1" ]]; then
  OUT_NAME="line_follow_model_live_drive"
fi
if [[ "$SD_LOG" == "1" ]]; then
  OUT_NAME="${OUT_NAME}_sd"
fi

if [[ "$ENABLE_DRIVE" == "1" ]]; then
  MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
  PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-0}"
else
  MAX_LOOPS="${LINE_FOLLOW_MAX_LOOPS:-0}"
  PRINT_EVERY="${LINE_FOLLOW_PRINT_EVERY:-1}"
fi
LOOP_MS="${LINE_FOLLOW_LOOP_MS:-$LINE_FOLLOW_LOOP_MS_DEFAULT}"
SD_FILE="${LINE_FOLLOW_SD_FILE:-lf_log.bin}"
SD_AUTO_NAME="${LINE_FOLLOW_SD_AUTO_NAME:-0}"
SD_MAX_RECORDS="${LINE_FOLLOW_SD_MAX_RECORDS:-6000}"
SD_LOG_EVERY="${LINE_FOLLOW_SD_LOG_EVERY:-1}"
SD_FLUSH_EVERY="${LINE_FOLLOW_SD_FLUSH_EVERY:-64}"
HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-$LINE_FOLLOW_HIDDEN_SIZE_DEFAULT}"
NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-$LINE_FOLLOW_NUM_LAYERS_DEFAULT}"
MAX_WHEEL_SPEED_MPS="${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT}"
COMMAND_DEADBAND="${LINE_FOLLOW_COMMAND_DEADBAND:-$LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT}"
MIN_DRIVE_TICKS_PER_SEC="${LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC:-$LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT}"
QTI_MODE="${LINE_FOLLOW_QTI_MODE:-2}"
QTI_CHARGE_US="${QTI_CHARGE_US:-230}"
QTI_TIMEOUT_US="${QTI_TIMEOUT_US:-1000}"
QTI_SAMPLE_PERIOD_US="${QTI_SAMPLE_PERIOD_US:-0}"
MAX_WHEEL_TICKS_PER_SEC_Q1000="$(
  awk -v mps="$MAX_WHEEL_SPEED_MPS" 'BEGIN {
    printf "%d", (mps / (3.14159265358979323846 * 0.065)) * 64.0 * 1000.0 + 0.5
  }'
)"
COMMAND_DEADBAND_Q1000="$(
  awk -v deadband="$COMMAND_DEADBAND" 'BEGIN { printf "%d", deadband * 1000.0 + 0.5 }'
)"

if [[ "$SD_LOG" == "1" && -z "${LINE_FOLLOW_SD_AUTO_NAME+x}" ]]; then
  SD_AUTO_NAME=1
fi

OUT_DIR="$LINE_FOLLOW_ROOT/build/line-follow"
SRC="$LINE_FOLLOW_ROOT/firmware/line_follow_model_live.c"
QTI_COG_SRC="$LINE_FOLLOW_ROOT/firmware/line_follow_qti_sampler.s"
QTI_COG_OBJ="$OUT_DIR/line_follow_qti_sampler.o"
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
  --hidden-size "$HIDDEN_SIZE" \
  --num-layers "$NUM_LAYERS" \
  "$weights" \
  "$HEADER"

args=(
  -Os -mcmm -m32bit-doubles -fno-exceptions -std=c99
  -DLINE_FOLLOW_ENABLE_DRIVE="$ENABLE_DRIVE"
  -DLINE_FOLLOW_MAX_LOOPS="$MAX_LOOPS"
  -DLINE_FOLLOW_LOOP_MS="$LOOP_MS"
  -DLINE_FOLLOW_PRINT_EVERY="$PRINT_EVERY"
  -DLINE_FOLLOW_SD_LOG="$SD_LOG"
  -DLINE_FOLLOW_SD_FILE="\"$SD_FILE\""
  -DLINE_FOLLOW_SD_AUTO_NAME="$SD_AUTO_NAME"
  -DLINE_FOLLOW_SD_MAX_RECORDS="$SD_MAX_RECORDS"
  -DLINE_FOLLOW_SD_LOG_EVERY="$SD_LOG_EVERY"
  -DLINE_FOLLOW_SD_FLUSH_EVERY="$SD_FLUSH_EVERY"
  -DLINE_FOLLOW_SD_DO_PIN="${LINE_FOLLOW_SD_DO_PIN:-22}"
  -DLINE_FOLLOW_SD_CLK_PIN="${LINE_FOLLOW_SD_CLK_PIN:-23}"
  -DLINE_FOLLOW_SD_DI_PIN="${LINE_FOLLOW_SD_DI_PIN:-24}"
  -DLINE_FOLLOW_SD_CS_PIN="${LINE_FOLLOW_SD_CS_PIN:-25}"
  -DHIDDEN_SIZE="$HIDDEN_SIZE"
  -DNUM_LAYERS="$NUM_LAYERS"
  -DMAX_WHEEL_SPEED_MPS="$MAX_WHEEL_SPEED_MPS"
  -DCOMMAND_DEADBAND="$COMMAND_DEADBAND"
  -DCOMMAND_DEADBAND_Q1000="$COMMAND_DEADBAND_Q1000"
  -DMIN_DRIVE_TICKS_PER_SEC="$MIN_DRIVE_TICKS_PER_SEC"
  -DMAX_WHEEL_TICKS_PER_SEC_Q1000="$MAX_WHEEL_TICKS_PER_SEC_Q1000"
  -DQTI_LEFT_PIN="${QTI_LEFT_PIN:-7}"
  -DQTI_MIDDLE_PIN="${QTI_MIDDLE_PIN:-6}"
  -DQTI_RIGHT_PIN="${QTI_RIGHT_PIN:-5}"
  -DQTI_WHITE_TIME="${QTI_WHITE_TIME:-$LINE_FOLLOW_QTI_WHITE_TIME_DEFAULT}"
  -DQTI_BLACK_TIME="${QTI_BLACK_TIME:-$LINE_FOLLOW_QTI_BLACK_TIME_DEFAULT}"
  -DQTI_THRESHOLD_Q1000="${QTI_THRESHOLD_Q1000:-500}"
  -DLINE_FOLLOW_QTI_MODE="$QTI_MODE"
  -DQTI_CHARGE_US="$QTI_CHARGE_US"
  -DQTI_TIMEOUT_US="$QTI_TIMEOUT_US"
  -DQTI_SAMPLE_PERIOD_US="$QTI_SAMPLE_PERIOD_US"
  -I"$OUT_DIR"
  -I"$LINE_FOLLOW_ROOT/firmware"
  -I"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools"
  -I"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext"
  -I"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c"
  -L"$PARALLAX_SIMPLE_LIBS/Utility/libsimpletools/cmm"
  -L"$PARALLAX_SIMPLE_LIBS/TextDevices/libsimpletext/cmm"
  -L"$PARALLAX_SIMPLE_LIBS/Protocol/libsimplei2c/cmm"
)

if [[ -n "${LINE_FOLLOW_INFERENCE_MODE:-}" ]]; then
  args+=(-DLINE_FOLLOW_INFERENCE_MODE="$LINE_FOLLOW_INFERENCE_MODE")
fi

libs=(-lsimpletools -lsimpletext -lsimplei2c -lm)
extra_objects=()

if [[ "$QTI_MODE" == "2" ]]; then
  propeller-elf-gcc -c -o "$QTI_COG_OBJ" "$QTI_COG_SRC"
  extra_objects+=("$QTI_COG_OBJ")
fi

if [[ "$ENABLE_DRIVE" == "1" ]]; then
  args+=(
    -I"$PARALLAX_SIMPLE_LIBS/Robotics/ActivityBot/libabdrive"
    -I"$PARALLAX_SIMPLE_LIBS/TextDevices/libfdserial"
    -L"$PARALLAX_SIMPLE_LIBS/Robotics/ActivityBot/libabdrive/cmm"
    -L"$PARALLAX_SIMPLE_LIBS/TextDevices/libfdserial/cmm"
  )
  libs=(-labdrive -lfdserial "${libs[@]}")
fi

propeller-elf-gcc "${args[@]}" -o "$OUT" "$SRC" "${extra_objects[@]}" "${libs[@]}"

propeller-elf-size "$OUT"
echo "Built $OUT"
echo "Embedded checkpoint: $weights"
echo "Header: $HEADER"
echo "Loop pause: ${LOOP_MS} ms"
if [[ "$PRINT_EVERY" == "0" ]]; then
  echo "Print every: disabled"
else
  echo "Print every: $PRINT_EVERY loop(s)"
fi
echo "Hidden size: $HIDDEN_SIZE"
echo "Num recurrent layers: $NUM_LAYERS"
echo "Inference mode: ${LINE_FOLLOW_INFERENCE_MODE:-auto}"
echo "QTI mode: $QTI_MODE (0=sequential, 1=grouped C, 2=PASM cog)"
echo "QTI charge: $QTI_CHARGE_US us"
echo "QTI timeout: $QTI_TIMEOUT_US us"
echo "QTI sample period: $QTI_SAMPLE_PERIOD_US us"
echo "Max wheel speed: $MAX_WHEEL_SPEED_MPS m/s"
echo "Command deadband: $COMMAND_DEADBAND"
echo "Minimum drive ticks/sec: $MIN_DRIVE_TICKS_PER_SEC"
if [[ "$SD_LOG" == "1" ]]; then
  echo "SD logging is ENABLED: file=$SD_FILE auto_name=$SD_AUTO_NAME max_records=$SD_MAX_RECORDS log_every=$SD_LOG_EVERY flush_every=$SD_FLUSH_EVERY"
else
  echo "SD logging is disabled"
fi
if [[ "$ENABLE_DRIVE" == "1" ]]; then
  if [[ "$MAX_LOOPS" == "0" ]]; then
    echo "Drive output is ENABLED with no loop cap. Use only with the robot safely staged."
  else
    echo "Drive output is ENABLED for $MAX_LOOPS loops. Use only with the robot safely staged."
  fi
else
  echo "Drive output is disabled. This build prints live QTI/model telemetry only."
fi
