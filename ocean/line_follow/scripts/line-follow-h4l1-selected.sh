#!/usr/bin/env bash

# Shared selected-candidate defaults for the H4/L1 fixed recurrent workflow.
# Source parallax-env.sh before this file so SIM2REAL_ROOT is available.

LINE_FOLLOW_H4L1_SELECTED_RUN_ID="${LINE_FOLLOW_H4L1_SELECTED_RUN_ID:-h85dqfhx}"
LINE_FOLLOW_H4L1_SELECTED_CHECKPOINT="${LINE_FOLLOW_H4L1_SELECTED_CHECKPOINT:-$SIM2REAL_ROOT/checkpoints/line_follow/h85dqfhx/0000000084148224.bin}"

line_follow_h4l1_resolve_checkpoint() {
  RUN_ID="${LINE_FOLLOW_RUN_ID:-$LINE_FOLLOW_H4L1_SELECTED_RUN_ID}"
  if [[ -n "${LINE_FOLLOW_WEIGHTS:-}" ]]; then
    CHECKPOINT="$LINE_FOLLOW_WEIGHTS"
  elif [[ -n "${LINE_FOLLOW_RUN_ID:-}" ]]; then
    CHECKPOINT="$(
      find "$SIM2REAL_ROOT/checkpoints/line_follow/$RUN_ID" -maxdepth 1 -type f -name '*.bin' \
        -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-
    )"
  else
    CHECKPOINT="$LINE_FOLLOW_H4L1_SELECTED_CHECKPOINT"
  fi

  if [[ -n "${LINE_FOLLOW_WEIGHTS:-}" && -z "${LINE_FOLLOW_RUN_ID:-}" ]]; then
    local parent
    parent="$(basename "$(dirname "$CHECKPOINT")")"
    if [[ -n "$parent" && "$parent" != "." && "$parent" != "/" ]]; then
      RUN_ID="$parent"
    fi
  fi
}

line_follow_h4l1_apply_run_deploy_env() {
  if [[ "${LINE_FOLLOW_MATCH_RUN_DEPLOY_ENV:-1}" == "0" ]]; then
    return
  fi

  local run_log="$SIM2REAL_ROOT/logs/line_follow/$RUN_ID.json"
  if [[ ! -f "$run_log" ]]; then
    echo "No run log for $RUN_ID; using config/env deploy settings."
    return
  fi

  local helper="$LINE_FOLLOW_ROOT/scripts/line-follow-run-deploy-env.py"
  local key value
  echo "Applying deploy-sensitive env from $run_log"
  while IFS='=' read -r key value; do
    case "$key" in
      LINE_FOLLOW_GATE_DT|LINE_FOLLOW_GATE_DT_MIN|LINE_FOLLOW_GATE_DT_MAX|\
      LINE_FOLLOW_MAX_WHEEL_SPEED_MPS|LINE_FOLLOW_COMMAND_DEADBAND|\
      LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC|QTI_WHITE_TIME|QTI_BLACK_TIME|\
      QTI_THRESHOLD_Q1000)
        if [[ -z "${!key+x}" ]]; then
          export "$key=$value"
          echo "  $key=$value"
        else
          echo "  $key=${!key} (env override)"
        fi
        ;;
    esac
  done < <("$SIM2REAL_ROOT/.venv/bin/python" "$helper" "$run_log")
}
