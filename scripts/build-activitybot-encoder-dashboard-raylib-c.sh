#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/host/activitybot_encoder_dashboard_raylib.c"
OUT="$ROOT/build/activitybot-encoder-dashboard-raylib-c"
RAYLIB_ROOT="$ROOT/raylib-5.5_linux_amd64"
RAYLIB_A="$RAYLIB_ROOT/lib/libraylib.a"

if [[ ! -f "$RAYLIB_A" ]]; then
  echo "Missing $RAYLIB_A"
  echo "Run ./build.sh breakout --fast once, or restore the PufferLib raylib bundle."
  exit 1
fi

mkdir -p "$(dirname "$OUT")"

CC="${CC:-clang}"
"$CC" \
  -std=c99 -O2 -Wall -Wextra -Werror=return-type \
  -I"$RAYLIB_ROOT/include" \
  -DPLATFORM_DESKTOP \
  -DSIM2REAL_ROOT="\"$ROOT\"" \
  "$SRC" \
  "$RAYLIB_A" \
  -lGL -lm -lpthread -ldl -lrt -lX11 \
  -o "$OUT"

echo "Built $OUT"
