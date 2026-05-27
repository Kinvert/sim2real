#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

PORT="${1:-${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}}"
ELF="$LINE_FOLLOW_ROOT/build/parallax-smoke/hello.elf"

if [[ ! -f "$ELF" ]]; then
  "$LINE_FOLLOW_ROOT/scripts/parallax-build-smoke.sh"
fi

echo "Loading $ELF to RAM on $PORT using board $PROPELLER_LOAD_BOARD"
echo "This does not write EEPROM. Press Ctrl-C to leave terminal mode."
exec propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -r -t "$ELF"
