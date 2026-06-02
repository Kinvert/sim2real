#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

ENABLE_DRIVE=0
SD_LOG=0
OUT_NAME="line_follow_model_live"
PIN_PROFILE="${LINE_FOLLOW_QTI_PIN_PROFILE:-default}"

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
    --pin-profile)
      if [[ $# -lt 2 ]]; then
        echo "--pin-profile requires p765 or p654" >&2
        exit 2
      fi
      PIN_PROFILE="$2"
      shift 2
      ;;
    *)
      echo "Usage: $0 [--drive] [--sd-log] [--pin-profile p765|p654]" >&2
      exit 2
      ;;
  esac
done

case "$PIN_PROFILE" in
  default|p765)
    DEFAULT_QTI_LEFT_PIN=7
    DEFAULT_QTI_MIDDLE_PIN=6
    DEFAULT_QTI_RIGHT_PIN=5
    DEFAULT_QTI_FAR_RIGHT_PIN=4
    DEFAULT_QTI_MAP_MODE=0
    ;;
  p654|right3)
    DEFAULT_QTI_LEFT_PIN=6
    DEFAULT_QTI_MIDDLE_PIN=5
    DEFAULT_QTI_RIGHT_PIN=4
    DEFAULT_QTI_FAR_RIGHT_PIN=4
    DEFAULT_QTI_MAP_MODE=0
    ;;
  adjacent4|p7654-adjacent)
    DEFAULT_QTI_LEFT_PIN=7
    DEFAULT_QTI_MIDDLE_PIN=6
    DEFAULT_QTI_RIGHT_PIN=5
    DEFAULT_QTI_FAR_RIGHT_PIN=4
    DEFAULT_QTI_MAP_MODE=1
    ;;
  *)
    echo "Unknown --pin-profile '$PIN_PROFILE' (expected p765, p654, or adjacent4)" >&2
    exit 2
    ;;
esac

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
RUN_MS="${LINE_FOLLOW_RUN_MS:-0}"
START_DELAY_MS="${LINE_FOLLOW_START_DELAY_MS:-0}"
STARTUP_PRINTS="${LINE_FOLLOW_STARTUP_PRINTS:-1}"
STATUS_LEDS="${LINE_FOLLOW_STATUS_LEDS:-0}"
SD_FILE="${LINE_FOLLOW_SD_FILE:-lf_log.bin}"
SD_PREV_FILE="${LINE_FOLLOW_SD_PREV_FILE:-lfprev.bin}"
SD_BACKUP_PREVIOUS="${LINE_FOLLOW_SD_BACKUP_PREVIOUS:-0}"
SD_AUTO_NAME="${LINE_FOLLOW_SD_AUTO_NAME:-0}"
SD_MAX_RECORDS="${LINE_FOLLOW_SD_MAX_RECORDS:-6000}"
SD_LOG_EVERY="${LINE_FOLLOW_SD_LOG_EVERY:-1}"
SD_FLUSH_EVERY="${LINE_FOLLOW_SD_FLUSH_EVERY:-64}"
SD_CHUNK_RECORDS="${LINE_FOLLOW_SD_CHUNK_RECORDS:-0}"
SD_DEFERRED="${LINE_FOLLOW_SD_DEFERRED:-0}"
SD_COG_LOG="${LINE_FOLLOW_SD_COG_LOG:-0}"
SD_COG_STACK_WORDS="${LINE_FOLLOW_SD_COG_STACK_WORDS:-160}"
SD_WRITE_TIMEOUT_MS="${LINE_FOLLOW_SD_WRITE_TIMEOUT_MS:-15000}"
BUILD_ID="${LINE_FOLLOW_BUILD_ID:-$(date +%H%M%S)}"
if [[ "$BUILD_ID" =~ ^[0-9]+$ ]]; then
  BUILD_ID="$((10#$BUILD_ID))"
fi
if [[ "$SD_LOG" == "1" && "$ENABLE_DRIVE" == "1" ]]; then
  SD_REQUIRED="${LINE_FOLLOW_SD_REQUIRED:-0}"
  SD_STOP_ON_WRITE_FAIL="${LINE_FOLLOW_SD_STOP_ON_WRITE_FAIL:-0}"
else
  SD_REQUIRED="${LINE_FOLLOW_SD_REQUIRED:-1}"
  SD_STOP_ON_WRITE_FAIL="${LINE_FOLLOW_SD_STOP_ON_WRITE_FAIL:-$SD_REQUIRED}"
fi
HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-$LINE_FOLLOW_HIDDEN_SIZE_DEFAULT}"
NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-$LINE_FOLLOW_NUM_LAYERS_DEFAULT}"
MAX_WHEEL_SPEED_MPS="${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT}"
COMMAND_DEADBAND="${LINE_FOLLOW_COMMAND_DEADBAND:-$LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT}"
MIN_DRIVE_TICKS_PER_SEC="${LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC:-$LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT}"
QTI_MODE="${LINE_FOLLOW_QTI_MODE:-2}"
QTI_MAP_MODE="${LINE_FOLLOW_QTI_MAP_MODE:-$DEFAULT_QTI_MAP_MODE}"
QTI_CHARGE_US="${QTI_CHARGE_US:-1000}"
QTI_TIMEOUT_US="${QTI_TIMEOUT_US:-1000}"
QTI_SAMPLE_PERIOD_US="${QTI_SAMPLE_PERIOD_US:-0}"
OBS_SUBTRACT_MIN="${LINE_FOLLOW_OBS_SUBTRACT_MIN:-0}"
OBS_CONTRAST_GAIN_Q1000="${LINE_FOLLOW_OBS_CONTRAST_GAIN_Q1000:-1000}"
OBS_RAW_DIFF="${LINE_FOLLOW_OBS_RAW_DIFF:-$LINE_FOLLOW_OBS_RAW_DIFF_DEFAULT}"
OBS_RAW_DIFF_GAIN_Q1000="${LINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000:-$LINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000_DEFAULT}"
OBS_RAW_DIFF_SPAN="${LINE_FOLLOW_OBS_RAW_DIFF_SPAN:-$LINE_FOLLOW_OBS_RAW_DIFF_SPAN_DEFAULT}"
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
QTI4_COG_SRC="$LINE_FOLLOW_ROOT/firmware/line_follow_qti_sampler4.s"
QTI4_COG_OBJ="$OUT_DIR/line_follow_qti_sampler4.o"
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
  -Os -ffunction-sections -fdata-sections -mcmm -m32bit-doubles -fno-exceptions -std=c99
  -DLINE_FOLLOW_ENABLE_DRIVE="$ENABLE_DRIVE"
  -DLINE_FOLLOW_MAX_LOOPS="$MAX_LOOPS"
  -DLINE_FOLLOW_RUN_MS="$RUN_MS"
  -DLINE_FOLLOW_LOOP_MS="$LOOP_MS"
  -DLINE_FOLLOW_PRINT_EVERY="$PRINT_EVERY"
  -DLINE_FOLLOW_START_DELAY_MS="$START_DELAY_MS"
  -DLINE_FOLLOW_STARTUP_PRINTS="$STARTUP_PRINTS"
  -DLINE_FOLLOW_STATUS_LEDS="$STATUS_LEDS"
  -DLINE_FOLLOW_SD_LOG="$SD_LOG"
  -DLINE_FOLLOW_SD_FILE="\"$SD_FILE\""
  -DLINE_FOLLOW_SD_PREV_FILE="\"$SD_PREV_FILE\""
  -DLINE_FOLLOW_SD_BACKUP_PREVIOUS="$SD_BACKUP_PREVIOUS"
  -DLINE_FOLLOW_SD_AUTO_NAME="$SD_AUTO_NAME"
  -DLINE_FOLLOW_SD_REQUIRED="$SD_REQUIRED"
  -DLINE_FOLLOW_SD_STOP_ON_WRITE_FAIL="$SD_STOP_ON_WRITE_FAIL"
  -DLINE_FOLLOW_SD_MAX_RECORDS="$SD_MAX_RECORDS"
  -DLINE_FOLLOW_SD_LOG_EVERY="$SD_LOG_EVERY"
  -DLINE_FOLLOW_SD_FLUSH_EVERY="$SD_FLUSH_EVERY"
  -DLINE_FOLLOW_SD_CHUNK_RECORDS="$SD_CHUNK_RECORDS"
  -DLINE_FOLLOW_SD_DEFERRED="$SD_DEFERRED"
  -DLINE_FOLLOW_SD_COG_LOG="$SD_COG_LOG"
  -DLINE_FOLLOW_SD_COG_STACK_WORDS="$SD_COG_STACK_WORDS"
  -DLINE_FOLLOW_SD_WRITE_TIMEOUT_MS="$SD_WRITE_TIMEOUT_MS"
  -DLINE_FOLLOW_BUILD_ID="$BUILD_ID"
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
  -DQTI_LEFT_PIN="${QTI_LEFT_PIN:-$DEFAULT_QTI_LEFT_PIN}"
  -DQTI_MIDDLE_PIN="${QTI_MIDDLE_PIN:-$DEFAULT_QTI_MIDDLE_PIN}"
  -DQTI_RIGHT_PIN="${QTI_RIGHT_PIN:-$DEFAULT_QTI_RIGHT_PIN}"
  -DQTI_FAR_RIGHT_PIN="${QTI_FAR_RIGHT_PIN:-$DEFAULT_QTI_FAR_RIGHT_PIN}"
  -DLINE_FOLLOW_QTI_MAP_MODE="$QTI_MAP_MODE"
  -DQTI_WHITE_TIME="${QTI_WHITE_TIME:-$LINE_FOLLOW_QTI_WHITE_TIME_DEFAULT}"
  -DQTI_BLACK_TIME="${QTI_BLACK_TIME:-$LINE_FOLLOW_QTI_BLACK_TIME_DEFAULT}"
  -DQTI_THRESHOLD_Q1000="${QTI_THRESHOLD_Q1000:-500}"
  -DLINE_FOLLOW_QTI_MODE="$QTI_MODE"
  -DQTI_CHARGE_US="$QTI_CHARGE_US"
  -DQTI_TIMEOUT_US="$QTI_TIMEOUT_US"
  -DQTI_SAMPLE_PERIOD_US="$QTI_SAMPLE_PERIOD_US"
  -DLINE_FOLLOW_OBS_SUBTRACT_MIN="$OBS_SUBTRACT_MIN"
  -DLINE_FOLLOW_OBS_CONTRAST_GAIN_Q1000="$OBS_CONTRAST_GAIN_Q1000"
  -DLINE_FOLLOW_OBS_RAW_DIFF="$OBS_RAW_DIFF"
  -DLINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000="$OBS_RAW_DIFF_GAIN_Q1000"
  -DLINE_FOLLOW_OBS_RAW_DIFF_SPAN="$OBS_RAW_DIFF_SPAN"
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

if [[ "$QTI_MODE" == "2" && "$QTI_MAP_MODE" == "0" ]]; then
  propeller-elf-gcc -c -o "$QTI_COG_OBJ" "$QTI_COG_SRC"
  extra_objects+=("$QTI_COG_OBJ")
elif [[ "$QTI_MODE" == "2" && "$QTI_MAP_MODE" != "0" ]]; then
  propeller-elf-gcc -c -o "$QTI4_COG_OBJ" "$QTI4_COG_SRC"
  extra_objects+=("$QTI4_COG_OBJ")
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

propeller-elf-gcc "${args[@]}" -Wl,--gc-sections -o "$OUT" "$SRC" "${extra_objects[@]}" "${libs[@]}"

propeller-elf-size "$OUT"
echo "Built $OUT"
echo "Embedded checkpoint: $weights"
echo "Header: $HEADER"
echo "Loop pause: ${LOOP_MS} ms"
echo "Run duration cap: ${RUN_MS} ms"
echo "Start delay: ${START_DELAY_MS} ms"
echo "Startup prints: ${STARTUP_PRINTS}"
echo "Status LEDs: ${STATUS_LEDS}"
echo "Build ID: ${BUILD_ID}"
if [[ "$PRINT_EVERY" == "0" ]]; then
  echo "Print every: disabled"
else
  echo "Print every: $PRINT_EVERY loop(s)"
fi
echo "Hidden size: $HIDDEN_SIZE"
echo "Num recurrent layers: $NUM_LAYERS"
echo "Inference mode: ${LINE_FOLLOW_INFERENCE_MODE:-auto}"
echo "QTI pin profile: $PIN_PROFILE"
echo "QTI pins: left=${QTI_LEFT_PIN:-$DEFAULT_QTI_LEFT_PIN} middle=${QTI_MIDDLE_PIN:-$DEFAULT_QTI_MIDDLE_PIN} right=${QTI_RIGHT_PIN:-$DEFAULT_QTI_RIGHT_PIN} far_right=${QTI_FAR_RIGHT_PIN:-$DEFAULT_QTI_FAR_RIGHT_PIN}"
echo "QTI mode: $QTI_MODE (0=sequential, 1=grouped C, 2=PASM cog)"
echo "QTI map mode: $QTI_MAP_MODE (0=three-pin, 1=four-adjacent-max, 2=four-center-max)"
echo "QTI charge: $QTI_CHARGE_US us"
echo "QTI timeout: $QTI_TIMEOUT_US us"
echo "QTI sample period: $QTI_SAMPLE_PERIOD_US us"
echo "Observation subtract-min: $OBS_SUBTRACT_MIN"
echo "Observation contrast gain q1000: $OBS_CONTRAST_GAIN_Q1000"
echo "Observation raw-diff: $OBS_RAW_DIFF"
echo "Observation raw-diff gain q1000: $OBS_RAW_DIFF_GAIN_Q1000"
echo "Observation raw-diff span: $OBS_RAW_DIFF_SPAN"
echo "Max wheel speed: $MAX_WHEEL_SPEED_MPS m/s"
echo "Command deadband: $COMMAND_DEADBAND"
echo "Minimum drive ticks/sec: $MIN_DRIVE_TICKS_PER_SEC"
if [[ "$SD_LOG" == "1" ]]; then
  echo "SD logging is ENABLED: file=$SD_FILE prev_file=$SD_PREV_FILE backup_previous=$SD_BACKUP_PREVIOUS auto_name=$SD_AUTO_NAME required=$SD_REQUIRED stop_on_write_fail=$SD_STOP_ON_WRITE_FAIL max_records=$SD_MAX_RECORDS log_every=$SD_LOG_EVERY flush_every=$SD_FLUSH_EVERY chunk_records=$SD_CHUNK_RECORDS deferred=$SD_DEFERRED cog_log=$SD_COG_LOG logger_stack_words=$SD_COG_STACK_WORDS write_timeout_ms=$SD_WRITE_TIMEOUT_MS"
else
  echo "SD logging is disabled"
fi
if [[ "$ENABLE_DRIVE" == "1" ]]; then
  if [[ "$MAX_LOOPS" == "0" && "$RUN_MS" == "0" ]]; then
    echo "Drive output is ENABLED with no loop cap. Use only with the robot safely staged."
  elif [[ "$RUN_MS" != "0" ]]; then
    echo "Drive output is ENABLED with run cap ${RUN_MS} ms. Use only with the robot safely staged."
  else
    echo "Drive output is ENABLED for $MAX_LOOPS loops. Use only with the robot safely staged."
  fi
else
  echo "Drive output is disabled. This build prints live QTI/model telemetry only."
fi
