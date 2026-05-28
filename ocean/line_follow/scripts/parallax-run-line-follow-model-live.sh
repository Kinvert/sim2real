#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

PORT="${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}"
ENABLE_DRIVE=0
OUT_NAME="line_follow_model_live"
LOG_FILE=""
HOST_TIMESTAMPS=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --log)
      if [[ $# -lt 2 ]]; then
        echo "--log requires a path" >&2
        exit 2
      fi
      LOG_FILE="$2"
      shift 2
      ;;
    --drive)
      ENABLE_DRIVE=1
      OUT_NAME="line_follow_model_live_drive"
      shift
      ;;
    --host-timestamps)
      HOST_TIMESTAMPS=1
      shift
      ;;
    /dev/*)
      PORT="$1"
      shift
      ;;
    *)
      echo "Usage: $0 [--drive] [--host-timestamps] [--log path] [/dev/ttyUSB0]" >&2
      exit 2
      ;;
  esac
done

ELF="$LINE_FOLLOW_ROOT/build/line-follow/$OUT_NAME.elf"

if [[ ! -f "$ELF" ]]; then
  if [[ "$ENABLE_DRIVE" == "1" ]]; then
    "$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-live.sh" --drive
  else
    "$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-live.sh"
  fi
fi

echo "Loading $ELF to RAM on $PORT using board $PROPELLER_LOAD_BOARD"
if [[ "$ENABLE_DRIVE" == "1" ]]; then
  echo "WARNING: drive output is enabled. Stage the robot safely before continuing."
else
  echo "Drive output is disabled; this is live QTI/model telemetry only."
fi
echo "This reads real QTI sensors, runs the embedded model, prints raw/min/max/obs/action/ticks, and does not write EEPROM."
echo "Press Ctrl-C to leave terminal mode."

if [[ -n "$LOG_FILE" ]]; then
  mkdir -p "$(dirname "$LOG_FILE")"
  if [[ "$HOST_TIMESTAMPS" == "1" ]]; then
    propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -r -t "$ELF" 2>&1 \
      | perl -MTime::HiRes=time -ne '$|=1; if (/^L /) { printf "%.6f %s", time, $_ } else { print }' \
      | tee "$LOG_FILE"
  else
    propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -r -t "$ELF" 2>&1 | tee "$LOG_FILE"
  fi
else
  exec propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -r -t "$ELF"
fi
