#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../../.."

log_keys="$(grep -c 'dict_set(out' ocean/line_follow/binding.c)"
if [[ "$log_keys" -gt 31 ]]; then
  echo "line_follow my_log emits $log_keys keys; vecenv adds n and has capacity 32" >&2
  exit 1
fi
if grep -q 'dict_set(out, "n"' ocean/line_follow/binding.c; then
  echo 'line_follow my_log must not emit n; vecenv adds it after my_log' >&2
  exit 1
fi

mkdir -p ocean/line_follow/build/tests

cc=${CC:-clang}
"$cc" \
  -std=gnu99 \
  -Wall \
  -Wextra \
  -Werror \
  -DLINE_FOLLOW_NO_RENDER \
  -I. \
  -Iocean/line_follow \
  ocean/line_follow/tests/test_line_follow.c \
  -lm \
  -o ocean/line_follow/build/tests/test_line_follow

ocean/line_follow/build/tests/test_line_follow
