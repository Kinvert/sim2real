#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

PORT="${1:-${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}}"
LEFT_TICKS="${LINE_FOLLOW_PROBE_LEFT_TICKS:-75}"
RIGHT_TICKS="${LINE_FOLLOW_PROBE_RIGHT_TICKS:-20}"
RUN_MS="${LINE_FOLLOW_PROBE_RUN_MS:-3000}"
PRINT_MS="${LINE_FOLLOW_PROBE_PRINT_MS:-250}"
OUT="$LINE_FOLLOW_ROOT/build/parallax-smoke/drive_ticks_probe_h4l1_${LEFT_TICKS}_${RIGHT_TICKS}.elf"
STAMP="${LINE_FOLLOW_PROBE_STAMP:-$(date +%Y%m%d-%H%M%S)}"
LOG_FILE="${LINE_FOLLOW_PROBE_LOG:-$LINE_FOLLOW_ROOT/build/parallax-smoke/drive-ticks-probe-h4l1-${LEFT_TICKS}-${RIGHT_TICKS}-${STAMP}.txt}"

DRIVE_LEFT_TICKS="$LEFT_TICKS" \
DRIVE_RIGHT_TICKS="$RIGHT_TICKS" \
DRIVE_RUN_MS="$RUN_MS" \
DRIVE_PRINT_MS="$PRINT_MS" \
  "$LINE_FOLLOW_ROOT/scripts/parallax-build-drive-ticks-probe.sh"

cp "$LINE_FOLLOW_ROOT/build/parallax-smoke/drive_ticks_probe.elf" "$OUT"

echo "H4/L1 drive-command probe loads to RAM only; EEPROM is not written."
echo "This intentionally commands motors briefly."
echo "Observed policy-equivalent command: left=$LEFT_TICKS right=$RIGHT_TICKS ticks/s"
echo "Run: ${RUN_MS} ms, print: ${PRINT_MS} ms"
echo "ELF: $OUT"
echo "Port: $PORT"
echo "Log: $LOG_FILE"

mkdir -p "$(dirname "$LOG_FILE")"

set +e
timeout "${LINE_FOLLOW_PROBE_TIMEOUT:-12s}" \
  propeller-load -b "$PROPELLER_LOAD_BOARD" -p "$PORT" -r -t "$OUT" \
  | tee "$LOG_FILE"
status=${PIPESTATUS[0]}
set -e

if [[ "$status" != "0" && "$status" != "124" ]]; then
  echo "Drive-command probe failed with status $status" >&2
  exit "$status"
fi
