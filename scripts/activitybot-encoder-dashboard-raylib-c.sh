#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"$ROOT/scripts/parallax-build-encoder-stream.sh"
"$ROOT/scripts/build-activitybot-encoder-dashboard-raylib-c.sh"

exec "$ROOT/build/activitybot-encoder-dashboard-raylib-c" "$@"
