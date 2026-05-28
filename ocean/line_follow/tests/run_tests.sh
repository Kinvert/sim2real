#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../../.."

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
