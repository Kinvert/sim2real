#!/usr/bin/env bash
set -euo pipefail

LINE_FOLLOW_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"$LINE_FOLLOW_ROOT/scripts/parallax-build-encoder-stream.sh"
"$LINE_FOLLOW_ROOT/scripts/build-activitybot-encoder-dashboard-raylib-c.sh"

exec "$LINE_FOLLOW_ROOT/build/activitybot-encoder-dashboard-raylib-c" "$@"
