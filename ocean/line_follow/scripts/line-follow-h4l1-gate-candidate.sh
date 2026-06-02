#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/parallax-env.sh"
source "$LINE_FOLLOW_ROOT/scripts/line-follow-h4l1-selected.sh"

line_follow_h4l1_resolve_checkpoint
line_follow_h4l1_apply_run_deploy_env

HIDDEN_SIZE="${LINE_FOLLOW_HIDDEN_SIZE:-4}"
NUM_LAYERS="${LINE_FOLLOW_NUM_LAYERS:-1}"
STAMP="${LINE_FOLLOW_GATE_STAMP:-$(date +%Y%m%d-%H%M%S)}"
GATE_DIR="${LINE_FOLLOW_GATE_DIR:-$LINE_FOLLOW_ROOT/build/line-follow/gate-h4l1-${RUN_ID}-${STAMP}}"
RANDOM_COUNT="${LINE_FOLLOW_GATE_RANDOM_COUNT:-20000}"
BIAS_WARN_TICKS="${LINE_FOLLOW_GATE_BIAS_WARN_TICKS:-40}"
MISMATCH_FRAC="${LINE_FOLLOW_GATE_FIXED_TICK_MISMATCH_FRAC:-0.15}"

DT="${LINE_FOLLOW_GATE_DT:-$(config_value dt 0.0165)}"
DT_MIN="${LINE_FOLLOW_GATE_DT_MIN:-$(config_value dt_min 0.015)}"
DT_MAX="${LINE_FOLLOW_GATE_DT_MAX:-$(config_value dt_max 0.020)}"
MAX_WHEEL_SPEED_MPS="${LINE_FOLLOW_MAX_WHEEL_SPEED_MPS:-$(config_value max_wheel_speed_mps 0.260)}"
COMMAND_DEADBAND="${LINE_FOLLOW_COMMAND_DEADBAND:-$(config_value command_deadband 0.0631585622560529)}"
MIN_DRIVE_TICKS_PER_SEC="${LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC:-$(config_value min_drive_ticks_per_sec 20)}"

mkdir -p "$GATE_DIR"
summary="$GATE_DIR/summary.txt"
exec > >(tee "$summary") 2>&1

echo "H4/L1 candidate gate"
echo "Run ID: $RUN_ID"
echo "Checkpoint: $CHECKPOINT"
echo "Checkpoint sha256: $(sha256sum "$CHECKPOINT" | awk '{print $1}')"
echo "Output dir: $GATE_DIR"
echo "Shape: hidden=$HIDDEN_SIZE layers=$NUM_LAYERS"
echo "Timing: dt=$DT dt_min=$DT_MIN dt_max=$DT_MAX"
echo "Drive scale: max_mps=$MAX_WHEEL_SPEED_MPS deadband=$COMMAND_DEADBAND min_ticks=$MIN_DRIVE_TICKS_PER_SEC"
echo

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "Missing H4/L1 checkpoint: $CHECKPOINT" >&2
  exit 1
fi

echo "== Center/balanced fixed recurrent bias =="
"$LINE_FOLLOW_ROOT/scripts/line-follow-center-bias.py" \
  "$CHECKPOINT" \
  --hidden-size "$HIDDEN_SIZE" \
  --num-layers "$NUM_LAYERS" \
  --steps 200 \
  --max-wheel-speed-mps "$MAX_WHEEL_SPEED_MPS" \
  --command-deadband "$COMMAND_DEADBAND" \
  --min-drive-ticks-per-sec "$MIN_DRIVE_TICKS_PER_SEC" \
  --warn-bias-ticks "$BIAS_WARN_TICKS" \
  | tee "$GATE_DIR/center-bias.txt"
echo

echo "== Deterministic policy benchmark =="
LINE_FOLLOW_WEIGHTS="$CHECKPOINT" \
LINE_FOLLOW_HIDDEN_SIZE="$HIDDEN_SIZE" \
LINE_FOLLOW_NUM_LAYERS="$NUM_LAYERS" \
  "$LINE_FOLLOW_ROOT/scripts/line-follow-policy-benchmark.sh" \
    --dt "$DT" \
    --dt-min "$DT_MIN" \
    --dt-max "$DT_MAX" \
    --max-wheel-speed-mps "$MAX_WHEEL_SPEED_MPS" \
    --command-deadband "$COMMAND_DEADBAND" \
    --min-drive-ticks-per-sec "$MIN_DRIVE_TICKS_PER_SEC" \
    | tee "$GATE_DIR/policy-benchmark.txt"
echo

echo "== Full float vs fixed recurrent equivalence =="
"$SIM2REAL_ROOT/.venv/bin/python" \
  "$LINE_FOLLOW_ROOT/scripts/line-follow-model-equivalence.py" \
  "$CHECKPOINT" \
  --hidden-size "$HIDDEN_SIZE" \
  --num-layers "$NUM_LAYERS" \
  --random-count "$RANDOM_COUNT" \
  --max-wheel-speed-mps "$MAX_WHEEL_SPEED_MPS" \
  --command-deadband "$COMMAND_DEADBAND" \
  --min-drive-ticks-per-sec "$MIN_DRIVE_TICKS_PER_SEC" \
  --max-fixed-tick-mismatch-frac "$MISMATCH_FRAC" \
  | tee "$GATE_DIR/model-equivalence.txt"
echo

echo "== Build-only deploy smoke =="
LINE_FOLLOW_FLASH=0 \
LINE_FOLLOW_DEPLOY_TAG="gate-${STAMP}" \
LINE_FOLLOW_WEIGHTS="$CHECKPOINT" \
LINE_FOLLOW_HIDDEN_SIZE="$HIDDEN_SIZE" \
LINE_FOLLOW_NUM_LAYERS="$NUM_LAYERS" \
  "$LINE_FOLLOW_ROOT/scripts/line-follow-h4l1-flash-deploy.sh" \
  "${1:-${PROPELLER_LOAD_PORT:-/dev/ttyUSB0}}" \
  | tee "$GATE_DIR/deploy-build.txt"
echo

echo "Gate complete."
echo "Summary: $summary"
