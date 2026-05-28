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
    row = {
        "step": step,
        "raw": [int(v) for v in parts[2:6]],
        "obs": [int(v) for v in parts[14:18]],
        "action": [float(v) for v in parts[18:20]],
        "ticks": [int(v) for v in parts[20:22]],
    }
    if len(parts) >= 23:
        row["dt_ms"] = int(parts[22])
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

    labels = ["outer_left", "inner_left", "inner_right", "outer_right"]
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
    dt_values = [row["dt_ms"] for row in rows if "dt_ms" in row and row["dt_ms"] > 0]
    if dt_values:
        print(
            f"loop_dt_ms min={min(dt_values)} p50={percentile(dt_values, 50):.1f} "
            f"p95={percentile(dt_values, 95):.1f} max={max(dt_values)} "
            f"mean={statistics.mean(dt_values):.1f}"
        )
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


if __name__ == "__main__":
    main()
