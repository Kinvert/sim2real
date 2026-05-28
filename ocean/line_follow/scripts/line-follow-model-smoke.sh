#!/usr/bin/env bash
set -euo pipefail

ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")/../../.." rev-parse --show-toplevel)"
source "$ROOT/ocean/line_follow/scripts/parallax-env.sh"
OUT="$ROOT/ocean/line_follow/build/line-follow/line_follow_model_smoke"
SRC="$ROOT/ocean/line_follow/host/line_follow_model_smoke.c"

mkdir -p "$(dirname "$OUT")"

weights="${LINE_FOLLOW_WEIGHTS:-}"
if [[ -z "$weights" ]]; then
  weights="$(find "$ROOT/checkpoints/line_follow" -type f -name '*.bin' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-)"
fi

if [[ -z "$weights" || ! -f "$weights" ]]; then
  echo "No line_follow checkpoint found. Set LINE_FOLLOW_WEIGHTS=/path/to/checkpoint.bin" >&2
  exit 1
fi

cc=${CC:-clang}
"$cc" -std=gnu99 -Wall -Wextra -Werror -Wno-unused-parameter \
  -DHIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-$LINE_FOLLOW_HIDDEN_SIZE_DEFAULT}" \
  -DNUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-$LINE_FOLLOW_NUM_LAYERS_DEFAULT}" \
  -DMAX_WHEEL_SPEED_MPS="${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$LINE_FOLLOW_MAX_WHEEL_SPEED_MPS_DEFAULT}" \
  -I"$ROOT" "$SRC" -lm -o "$OUT"

exec "$OUT" "$weights" "$@"
