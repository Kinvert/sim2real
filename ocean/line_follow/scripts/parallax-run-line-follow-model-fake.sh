#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

PORT="${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    /dev/*)
      PORT="$1"
      shift
      ;;
    *)
      echo "Usage: $0 [/dev/ttyUSB0]" >&2
      exit 2
      ;;
  esac
done

ELF="$LINE_FOLLOW_ROOT/build/line-follow/line_follow_model_fake.elf"

if [[ ! -f "$ELF" ]]; then
  "$LINE_FOLLOW_ROOT/scripts/parallax-build-line-follow-model-fake.sh"
fi

echo "Loading $ELF to RAM on $PORT using board $PROPELLER_LOAD_BOARD"
echo "This runs the embedded trained model on fake observations."
echo "No QTI reads, no motor commands, no EEPROM write. Press Ctrl-C to leave terminal mode."
exec propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -r -t "$ELF"
