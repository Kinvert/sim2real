#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"

echo "SIM2REAL_ROOT=$SIM2REAL_ROOT"
echo "PARALLAX_ROOT=$PARALLAX_ROOT"
echo "PROPELLER_LOAD_BOARD=$PROPELLER_LOAD_BOARD"
echo

propeller-elf-gcc --version | head -1
echo

echo "Loader help flags:"
propeller-load -h 2>&1 | sed -n '1,28p' || true
echo

echo "Linux serial devices visible to WSL:"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
echo

echo "Propeller loader port scan:"
propeller-load -Q || true
