#!/usr/bin/env python3
import json
import pathlib
import sys


def emit(name, value):
    if value is not None:
        print(f"{name}={value}")


def emit_int(name, value):
    if value is not None:
        print(f"{name}={int(round(float(value)))}")


def main():
    if len(sys.argv) != 2:
        print("usage: line-follow-run-deploy-env.py RUN_LOG.json", file=sys.stderr)
        return 2

    path = pathlib.Path(sys.argv[1])
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    env = data.get("env", {})
    emit("LINE_FOLLOW_GATE_DT", env.get("dt"))
    emit("LINE_FOLLOW_GATE_DT_MIN", env.get("dt_min"))
    emit("LINE_FOLLOW_GATE_DT_MAX", env.get("dt_max"))
    emit("LINE_FOLLOW_MAX_WHEEL_SPEED_MPS", env.get("max_wheel_speed_mps"))
    emit("LINE_FOLLOW_COMMAND_DEADBAND", env.get("command_deadband"))
    emit("LINE_FOLLOW_MIN_DRIVE_TICKS_PER_SEC", env.get("min_drive_ticks_per_sec"))
    emit_int("QTI_WHITE_TIME", env.get("qti_white_time"))
    emit_int("QTI_BLACK_TIME", env.get("qti_black_time"))
    threshold = env.get("qti_threshold")
    if threshold is not None:
        emit("QTI_THRESHOLD_Q1000", int(round(float(threshold) * 1000.0)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
