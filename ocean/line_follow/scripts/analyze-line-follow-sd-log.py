#!/usr/bin/env python3
import argparse
import csv
import os
import struct
from statistics import mean


MAGIC = 0x4453464C
HEADER_FMT = "<I18i128s"
ROW_FMT = "<17i"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
ROW_SIZE = struct.calcsize(ROW_FMT)

HEADER_NAMES = [
    "magic",
    "version",
    "header_size",
    "row_size",
    "sensor_count",
    "obs_size",
    "hidden_size",
    "num_layers",
    "clkfreq",
    "qti_white_time",
    "qti_black_time",
    "qti_threshold_q1000",
    "max_speed_mm_s",
    "min_drive_ticks_per_sec",
    "command_deadband_q1000",
    "loop_pause_ms",
    "sd_log_every",
    "sd_max_records",
    "model_raw_float_count",
    "checkpoint",
]

ROW_NAMES = [
    "loop",
    "elapsed_ms",
    "period_ms",
    "body_ms",
    "raw_left",
    "raw_middle",
    "raw_right",
    "encoder_left_ticks",
    "encoder_right_ticks",
    "obs_left_q1000",
    "obs_middle_q1000",
    "obs_right_q1000",
    "action_left_q1000",
    "action_right_q1000",
    "command_left_ticks",
    "command_right_ticks",
    "flags",
]


def percentile(values, pct):
    if not values:
        return 0
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * pct / 100.0))
    return ordered[max(0, min(idx, len(ordered) - 1))]


def read_log(path):
    with open(path, "rb") as f:
        header_bytes = f.read(HEADER_SIZE)
        if len(header_bytes) != HEADER_SIZE:
            raise SystemExit(f"{path}: too short for header")
        values = list(struct.unpack(HEADER_FMT, header_bytes))

        checkpoint_raw = values[-1]
        checkpoint = checkpoint_raw.split(b"\0", 1)[0].decode("utf-8", "replace")
        values[-1] = checkpoint
        header = dict(zip(HEADER_NAMES, values))

        if header["magic"] != MAGIC:
            raise SystemExit(f"{path}: bad magic 0x{header['magic']:08x}")
        if header["header_size"] != HEADER_SIZE:
            raise SystemExit(
                f"{path}: header size {header['header_size']} != parser {HEADER_SIZE}"
            )
        if header["row_size"] != ROW_SIZE:
            raise SystemExit(
                f"{path}: row size {header['row_size']} != parser {ROW_SIZE}"
            )

        rows = []
        while True:
            row_bytes = f.read(ROW_SIZE)
            if not row_bytes:
                break
            if len(row_bytes) != ROW_SIZE:
                print(f"warning: ignoring partial trailing row of {len(row_bytes)} bytes")
                break
            rows.append(dict(zip(ROW_NAMES, struct.unpack(ROW_FMT, row_bytes))))

    return header, rows


def add_derived(rows):
    prev = None
    for row in rows:
        if prev is None:
            row["delta_left_ticks"] = 0
            row["delta_right_ticks"] = 0
        else:
            row["delta_left_ticks"] = (
                row["encoder_left_ticks"] - prev["encoder_left_ticks"]
            )
            row["delta_right_ticks"] = (
                row["encoder_right_ticks"] - prev["encoder_right_ticks"]
            )
        row["avg_command_ticks"] = (
            row["command_left_ticks"] + row["command_right_ticks"]
        ) / 2.0
        row["avg_delta_ticks"] = (
            row["delta_left_ticks"] + row["delta_right_ticks"]
        ) / 2.0
        row["left_black"] = int(bool(row["flags"] & 1))
        row["middle_black"] = int(bool(row["flags"] & 2))
        row["right_black"] = int(bool(row["flags"] & 4))
        prev = row


def range_text(values):
    if not values:
        return "n/a"
    return f"{min(values)}..{max(values)}"


def summarize(header, rows, path):
    add_derived(rows)
    periods = [r["period_ms"] for r in rows[1:] if r["period_ms"] > 0]
    body = [r["body_ms"] for r in rows if r["body_ms"] > 0]
    left_cmd = [r["command_left_ticks"] for r in rows]
    right_cmd = [r["command_right_ticks"] for r in rows]
    avg_cmd = [r["avg_command_ticks"] for r in rows]
    min_ticks = header["min_drive_ticks_per_sec"]
    slow_rows = [r for r in rows if r["avg_command_ticks"] <= min_ticks + 1]
    zero_motion = [
        r for r in rows[1:] if r["delta_left_ticks"] == 0 and r["delta_right_ticks"] == 0
    ]

    print(f"path: {os.path.abspath(path)}")
    print(
        "checkpoint:",
        header["checkpoint"] if header["checkpoint"] else "(not recorded)",
    )
    print(
        "model:",
        f"hidden={header['hidden_size']} layers={header['num_layers']} "
        f"raw_floats={header['model_raw_float_count']}",
    )
    print(
        "firmware:",
        f"max_speed={header['max_speed_mm_s']} mm/s "
        f"min_ticks={header['min_drive_ticks_per_sec']} "
        f"deadband_q1000={header['command_deadband_q1000']} "
        f"qti_white={header['qti_white_time']} qti_black={header['qti_black_time']}",
    )
    print(
        "sd:",
        f"rows={len(rows)} log_every={header['sd_log_every']} "
        f"max_records={header['sd_max_records']} row_size={header['row_size']}",
    )

    if not rows:
        return

    duration_ms = sum(periods)
    wrapped_elapsed = rows[-1]["elapsed_ms"] < rows[0]["elapsed_ms"] or any(
        rows[i]["elapsed_ms"] < rows[i - 1]["elapsed_ms"] for i in range(1, len(rows))
    )
    print(f"duration: {duration_ms / 1000.0:.3f}s")
    if wrapped_elapsed:
        print("elapsed_ms: wrapped; duration uses summed period_ms")
    if periods:
        print(
            "period_ms:",
            f"avg={mean(periods):.2f} p50={percentile(periods, 50)} "
            f"p95={percentile(periods, 95)} max={max(periods)}",
        )
    if body:
        print(
            "body_ms:",
            f"avg={mean(body):.2f} p50={percentile(body, 50)} "
            f"p95={percentile(body, 95)} max={max(body)}",
        )

    print(
        "command_ticks:",
        f"left={range_text(left_cmd)} right={range_text(right_cmd)} "
        f"avg={mean(avg_cmd):.2f}",
    )
    print(
        "actions_q1000:",
        f"left={range_text([r['action_left_q1000'] for r in rows])} "
        f"right={range_text([r['action_right_q1000'] for r in rows])}",
    )
    print(
        "raw_qti:",
        f"left={range_text([r['raw_left'] for r in rows])} "
        f"middle={range_text([r['raw_middle'] for r in rows])} "
        f"right={range_text([r['raw_right'] for r in rows])}",
    )
    print(
        "obs_q1000:",
        f"left={range_text([r['obs_left_q1000'] for r in rows])} "
        f"middle={range_text([r['obs_middle_q1000'] for r in rows])} "
        f"right={range_text([r['obs_right_q1000'] for r in rows])}",
    )

    if duration_ms > 0 and len(rows) > 1:
        seconds = duration_ms / 1000.0
        left_total = rows[-1]["encoder_left_ticks"] - rows[0]["encoder_left_ticks"]
        right_total = rows[-1]["encoder_right_ticks"] - rows[0]["encoder_right_ticks"]
        print(
            "encoder_ticks:",
            f"left_total={left_total} right_total={right_total} "
            f"left_per_s={left_total / seconds:.2f} "
            f"right_per_s={right_total / seconds:.2f}",
        )

    print(
        "slow_rows:",
        f"{len(slow_rows)}/{len(rows)} ({100.0 * len(slow_rows) / len(rows):.1f}%) "
        f"avg_command <= min_ticks+1",
    )
    if len(rows) > 1:
        print(
            "zero_motion_rows:",
            f"{len(zero_motion)}/{len(rows) - 1} "
            f"({100.0 * len(zero_motion) / (len(rows) - 1):.1f}%)",
        )


def write_csv(rows, path):
    add_derived(rows)
    names = ROW_NAMES + [
        "delta_left_ticks",
        "delta_right_ticks",
        "avg_command_ticks",
        "avg_delta_ticks",
        "left_black",
        "middle_black",
        "right_black",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, names)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row[name] for name in names})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", help="Path to lf_log.bin copied from the ActivityBoard SD card")
    parser.add_argument("--csv", help="Optional CSV output path with decoded rows")
    args = parser.parse_args()

    header, rows = read_log(args.log)
    summarize(header, rows, args.log)
    if args.csv:
        write_csv(rows, args.csv)
        print(f"csv: {os.path.abspath(args.csv)}")


if __name__ == "__main__":
    main()
