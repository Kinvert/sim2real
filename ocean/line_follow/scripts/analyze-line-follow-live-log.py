#!/usr/bin/env python3
import argparse
import statistics


def parse_row(line):
    parts = line.strip().split()
    timestamp = None
    if parts:
        try:
            timestamp = float(parts[0])
            parts = parts[1:]
        except ValueError:
            pass
    if len(parts) < 22 or parts[0] != "L":
        return None
    try:
        step = int(parts[1])
    except ValueError:
        return None
    if len(parts) >= 25:
        row = {
            "step": step,
            "raw": [int(v) for v in parts[2:6]],
            "obs": [int(v) for v in parts[14:18]],
            "model_obs": [int(v) for v in parts[18:20]],
            "action": [float(v) for v in parts[20:22]],
            "ticks": [int(v) for v in parts[22:24]],
            "dt_ms": int(parts[24]),
        }
    else:
        row = {
            "step": step,
            "raw": [int(v) for v in parts[2:5]],
            "obs": [int(v) for v in parts[11:14]],
            "model_obs": [int(v) for v in parts[14:17]],
            "action": [float(v) for v in parts[17:19]],
            "ticks": [int(v) for v in parts[19:21]],
            "dt_ms": int(parts[21]),
        }
    if timestamp is not None:
        row["timestamp"] = timestamp
    return row


def percentile(values, pct):
    ordered = sorted(values)
    if not ordered:
        return 0.0
    pos = (len(ordered) - 1) * pct / 100.0
    lo = int(pos)
    hi = min(lo + 1, len(ordered) - 1)
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def summarize_column(rows, key, idx):
    values = [row[key][idx] for row in rows]
    return min(values), max(values), statistics.mean(values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    args = parser.parse_args()

    rows = []
    with open(args.log, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            row = parse_row(line)
            if row is not None:
                rows.append(row)

    if not rows:
        raise SystemExit("No live telemetry rows found")

    labels = ["left", "middle", "right"] if len(rows[0]["raw"]) == 3 else [
        "outer_left", "inner_left", "inner_right", "outer_right"]
    print(f"rows={len(rows)} first_step={rows[0]['step']} last_step={rows[-1]['step']}")
    timestamp_rows = [row for row in rows if "timestamp" in row]
    if len(timestamp_rows) >= 2:
        loop_ms = []
        for prev, curr in zip(timestamp_rows, timestamp_rows[1:]):
            step_delta = curr["step"] - prev["step"]
            if step_delta > 0:
                loop_ms.append(1000.0 * (curr["timestamp"] - prev["timestamp"]) / step_delta)
        if loop_ms:
            print(
                f"host_loop_ms min={min(loop_ms):.1f} p50={percentile(loop_ms, 50):.1f} "
                f"p95={percentile(loop_ms, 95):.1f} max={max(loop_ms):.1f} "
                f"mean={statistics.mean(loop_ms):.1f}"
            )
    raw_dt_values = [row["dt_ms"] for row in rows if "dt_ms" in row and row["dt_ms"] > 0]
    dt_values = [value for value in raw_dt_values if value <= 1000]
    if dt_values:
        print(
            f"loop_dt_ms min={min(dt_values)} p50={percentile(dt_values, 50):.1f} "
            f"p95={percentile(dt_values, 95):.1f} max={max(dt_values)} "
            f"mean={statistics.mean(dt_values):.1f}"
        )
    ignored_dt = len(raw_dt_values) - len(dt_values)
    if ignored_dt:
        print(f"ignored_implausible_loop_dt_ms={ignored_dt}")
    print("raw rc_time values: lower should be brighter/whiter, higher should be darker/blacker")
    for i, label in enumerate(labels):
        raw_values = [row["raw"][i] for row in rows]
        raw_min, raw_max, raw_mean = summarize_column(rows, "raw", i)
        obs_min, obs_max, obs_mean = summarize_column(rows, "obs", i)
        print(
            f"{label:12s} raw_min={raw_min:6d} raw_max={raw_max:6d} raw_mean={raw_mean:8.1f} "
            f"raw_p05={percentile(raw_values, 5):7.1f} raw_p50={percentile(raw_values, 50):7.1f} "
            f"raw_p95={percentile(raw_values, 95):7.1f} obs_min={obs_min:4d} "
            f"obs_max={obs_max:4d} obs_mean={obs_mean:6.1f}"
        )

    print("suggested first-pass per-sensor calibration from raw p05/p95:")
    for i, label in enumerate(labels):
        raw_values = [row["raw"][i] for row in rows]
        print(
            f"{label:12s} white={percentile(raw_values, 5):.0f} "
            f"black={percentile(raw_values, 95):.0f}"
        )

    for i, label in enumerate(["left", "right"]):
        action_min, action_max, action_mean = summarize_column(rows, "action", i)
        ticks_min, ticks_max, ticks_mean = summarize_column(rows, "ticks", i)
        clipped = sum(1 for row in rows if abs(row["action"][i]) >= 1.0)
        print(
            f"{label:12s} action_min={action_min:7.3f} action_max={action_max:7.3f} "
            f"action_mean={action_mean:7.3f} ticks_min={ticks_min:4d} "
            f"ticks_max={ticks_max:4d} ticks_mean={ticks_mean:7.1f} "
            f"clipped_action_rows={clipped}"
        )
    model_rows = [row for row in rows if "model_obs" in row]
    if model_rows:
        print("model observations used by policy:")
        model_labels = ["left", "middle", "right"] if len(model_rows[0]["model_obs"]) == 3 else [
            "inner_left", "inner_right"]
        for i, label in enumerate(model_labels):
            values = [row["model_obs"][i] for row in model_rows]
            print(
                f"{label:12s} obs_min={min(values):4d} obs_max={max(values):4d} "
                f"obs_mean={statistics.mean(values):6.1f}"
            )


if __name__ == "__main__":
    main()
