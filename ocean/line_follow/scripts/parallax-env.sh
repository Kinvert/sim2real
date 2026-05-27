#!/usr/bin/env bash

# Source this file before using the repo-local Parallax Propeller C tools.
LINE_FOLLOW_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM2REAL_ROOT="$(git -C "$LINE_FOLLOW_ROOT" rev-parse --show-toplevel)"
export LINE_FOLLOW_ROOT
export SIM2REAL_ROOT

export PARALLAX_ROOT="$SIM2REAL_ROOT/tools/parallax/simpleide/opt/parallax"
export PARALLAX_WORKSPACE="$PARALLAX_ROOT/Workspace/Learn"
export PARALLAX_SIMPLE_LIBS="$PARALLAX_WORKSPACE/Simple Libraries"

export PROPELLER_LOAD_BOARD="${PROPELLER_LOAD_BOARD:-activityboard}"
export PATH="$PARALLAX_ROOT/bin:$PATH"
