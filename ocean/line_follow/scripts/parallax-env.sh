#!/usr/bin/env bash

# Source this file before using the repo-local Parallax Propeller C tools.
LINE_FOLLOW_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM2REAL_ROOT="$(git -C "$LINE_FOLLOW_ROOT" rev-parse --show-toplevel)"
export LINE_FOLLOW_ROOT
export SIM2REAL_ROOT

export PARALLAX_ROOT="$SIM2REAL_ROOT/tools/parallax/simpleide/opt/parallax"
export PARALLAX_WORKSPACE="$PARALLAX_ROOT/Workspace/Learn"
export PARALLAX_SIMPLE_LIBS="$PARALLAX_WORKSPACE/Simple Libraries"

export PROPELLER_LOAD_BOARD="${PROPELLER_LOAD_BOARD:-activityboard}"
export PATH="$PARALLAX_ROOT/bin:$PATH"

config_int() {
  local key="$1"
  local fallback="$2"
  local value
  value="$(
    awk -F= -v key="$key" '
      {
        k = $1
        gsub(/^[ \t]+|[ \t]+$/, "", k)
        if (k == key) {
          v = $2
          sub(/[;#].*/, "", v)
          gsub(/^[ \t]+|[ \t]+$/, "", v)
          printf "%.0f", v
          exit
        }
      }
    ' "$SIM2REAL_ROOT/config/line_follow.ini"
  )"
  if [[ -n "$value" ]]; then
    printf "%s" "$value"
  else
    printf "%s" "$fallback"
  fi
}

config_ms() {
  local key="$1"
  local fallback="$2"
  local value
  value="$(
    awk -F= -v key="$key" '
      {
        k = $1
        gsub(/^[ \t]+|[ \t]+$/, "", k)
        if (k == key) {
          v = $2
          sub(/[;#].*/, "", v)
          gsub(/^[ \t]+|[ \t]+$/, "", v)
          printf "%.0f", v * 1000
          exit
        }
      }
    ' "$SIM2REAL_ROOT/config/line_follow.ini"
  )"
  if [[ -n "$value" ]]; then
    printf "%s" "$value"
  else
    printf "%s" "$fallback"
  fi
}

config_value() {
  local key="$1"
  local fallback="$2"
  local value
  value="$(
    awk -F= -v key="$key" '
      {
        k = $1
        gsub(/^[ \t]+|[ \t]+$/, "", k)
        if (k == key) {
          v = $2
          sub(/[;#].*/, "", v)
          gsub(/^[ \t]+|[ \t]+$/, "", v)
          printf "%s", v
          exit
        }
      }
    ' "$SIM2REAL_ROOT/config/line_follow.ini"
  )"
  if [[ -n "$value" ]]; then
    printf "%s" "$value"
  else
    printf "%s" "$fallback"
  fi
}

config_q1000() {
  local key="$1"
  local fallback="$2"
  local value
  value="$(
    awk -F= -v key="$key" '
      {
        k = $1
        gsub(/^[ \t]+|[ \t]+$/, "", k)
        if (k == key) {
          v = $2
          sub(/[;#].*/, "", v)
          gsub(/^[ \t]+|[ \t]+$/, "", v)
          printf "%.0f", v * 1000
          exit
        }
      }
    ' "$SIM2REAL_ROOT/config/line_follow.ini"
  )"
  if [[ -n "$value" ]]; then
    printf "%s" "$value"
  else
    printf "%s" "$fallback"
  fi
}

export LINE_FOLLOW_QTI_WHITE_TIME_DEFAULT
export LINE_FOLLOW_QTI_BLACK_TIME_DEFAULT
export LINE_FOLLOW_LOOP_MS_DEFAULT
export LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT
export LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT
export LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT
export LINE_FOLLOW_HIDDEN_SIZE_DEFAULT
export LINE_FOLLOW_NUM_LAYERS_DEFAULT
export LINE_FOLLOW_OBS_RAW_DIFF_DEFAULT
export LINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000_DEFAULT
export LINE_FOLLOW_OBS_RAW_DIFF_SPAN_DEFAULT
LINE_FOLLOW_QTI_WHITE_TIME_DEFAULT="$(config_int qti_white_time 40)"
LINE_FOLLOW_QTI_BLACK_TIME_DEFAULT="$(config_int qti_black_time 350)"
LINE_FOLLOW_LOOP_MS_DEFAULT="$(config_int firmware_loop_ms 0)"
LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT="$(config_value max_wheel_speed_mps 0.116)"
LINE_FOLLOW_COMMAND_DEADBAND_DEFAULT="$(config_value command_deadband 0.04)"
LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC_DEFAULT="$(config_int min_drive_ticks_per_sec 6)"
LINE_FOLLOW_HIDDEN_SIZE_DEFAULT="$(config_int hidden_size 8)"
LINE_FOLLOW_NUM_LAYERS_DEFAULT="$(config_int num_layers 0)"
LINE_FOLLOW_OBS_RAW_DIFF_DEFAULT="$(config_int obs_raw_diff 0)"
LINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000_DEFAULT="$(config_q1000 obs_raw_diff_gain 1000)"
LINE_FOLLOW_OBS_RAW_DIFF_SPAN_DEFAULT="$(config_int obs_raw_diff_span 0)"
