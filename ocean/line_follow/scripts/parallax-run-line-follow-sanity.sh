#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

PORT="${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}"
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
    /dev/*)
      PORT="$1"
      shift
      ;;
    *)
      echo "Usage: $0 [--fake|--drive] [/dev/ttyUSB0]" >&2
      exit 2
      ;;
  esac
done

if [[ "$FAKE_OBS" == "1" && "$ENABLE_DRIVE" == "1" ]]; then
  echo "--fake and --drive are intentionally mutually exclusive" >&2
  exit 2
fi

ELF="$LINE_FOLLOW_ROOT/build/line-follow/$OUT_NAME.elf"

if [[ ! -f "$ELF" ]]; then
  if [[ "$FAKE_OBS" == "1" ]]; then
    "$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-sanity.sh" --fake
  elif [[ "$ENABLE_DRIVE" == "1" ]]; then
    "$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-sanity.sh" --drive
  else
    "$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-sanity.sh"
  fi
fi

echo "Loading $ELF to RAM on $PORT using board $PROPELLER_LOAD_BOARD"
if [[ "$FAKE_OBS" == "1" ]]; then
  echo "Fake observation mode; no QTI reads and no motor commands."
elif [[ "$ENABLE_DRIVE" == "1" ]]; then
  echo "WARNING: drive output is enabled. Stage the robot safely before continuing."
else
  echo "Drive output is disabled; this is a QTI/policy telemetry run only."
fi
echo "This does not write EEPROM. Press Ctrl-C to leave terminal mode."
exec propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -r -t "$ELF"
