#!/usr/bin/env python3
import argparse
import csv
import os
import struct
from statistics import mean


MAGIC = 0x4453464C
HEADER_FMT = "<I18i128s"
HEADER_EXTRA_FMT_V2 = "<8i"
HEADER_EXTRA_FMT_V8 = "<12i"
HEADER_EXTRA_FMT_V9 = "<16i"
ROW_FMT = "<17i"
ROW_FMT_V2 = "<23i"
ROW_FMT_V3 = "<40i"
ROW_FMT_V7 = "<15h"
ROW_FMT_V8 = "<19h"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
HEADER_SIZE_V2 = HEADER_SIZE + struct.calcsize(HEADER_EXTRA_FMT_V2)
HEADER_SIZE_V8 = HEADER_SIZE + struct.calcsize(HEADER_EXTRA_FMT_V8)
HEADER_SIZE_V9 = HEADER_SIZE + struct.calcsize(HEADER_EXTRA_FMT_V9)
ROW_SIZE = struct.calcsize(ROW_FMT)
ROW_SIZE_V2 = struct.calcsize(ROW_FMT_V2)
ROW_SIZE_V3 = struct.calcsize(ROW_FMT_V3)
ROW_SIZE_V7 = struct.calcsize(ROW_FMT_V7)
ROW_SIZE_V8 = struct.calcsize(ROW_FMT_V8)

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

HEADER_EXTRA_NAMES_V2 = [
    "qti_mode",
    "qti_charge_us",
    "qti_timeout_us",
    "qti_sample_period_us",
    "start_delay_ms",
    "run_ms",
    "inference_mode",
    "build_id",
]

HEADER_EXTRA_NAMES_V8 = HEADER_EXTRA_NAMES_V2 + [
    "qti_map_mode",
    "obs_subtract_min",
    "obs_contrast_gain_q1000",
    "ramlog_reserved0",
]

HEADER_EXTRA_NAMES_V9 = HEADER_EXTRA_NAMES_V8 + [
    "obs_raw_diff",
    "obs_raw_diff_gain_q1000",
    "obs_raw_diff_span",
    "ramlog_reserved1",
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

ROW_NAMES_V2 = [
    "loop",
    "elapsed_ms",
    "period_ms",
    "body_ms",
    "period_us",
    "body_us",
    "qti_wait_us",
    "policy_us",
    "drive_us",
    "qti_sample_us",
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

ROW_NAMES_V3 = ROW_NAMES_V2 + [
    "diag_variant",
    "diag_phase",
    "hold_us",
    "prev_log_us",
    "qti_sample_period_us",
    "encoder_left_after",
    "encoder_right_after",
    "delta_left_ticks_explicit",
    "delta_right_ticks_explicit",
    "segment",
    "target_left_ticks",
    "target_right_ticks",
    "left_floor",
    "right_floor",
    "pair_state",
    "pair_min_q1000",
    "pair_max_q1000",
]

ROW_NAMES_V7 = [
    "loop",
    "elapsed_ms",
    "period_us",
    "raw_left",
    "raw_middle",
    "raw_right",
    "obs_left_q1000",
    "obs_middle_q1000",
    "obs_right_q1000",
    "action_left_q1000",
    "action_right_q1000",
    "command_left_ticks",
    "command_right_ticks",
    "flags",
    "ramlog_kind",
]

ROW_NAMES_V8 = [
    "loop",
    "elapsed_ms",
    "period_us",
    "raw_left",
    "raw_middle",
    "raw_right",
    "raw_physical_p7",
    "raw_physical_p6",
    "raw_physical_p5",
    "raw_physical_p4",
    "obs_left_q1000",
    "obs_middle_q1000",
    "obs_right_q1000",
    "action_left_q1000",
    "action_right_q1000",
    "command_left_ticks",
    "command_right_ticks",
    "flags",
    "ramlog_kind",
]


def expand_ramlog_row(row):
    period_us = row["period_us"]
    return {
        "loop": row["loop"],
        "elapsed_ms": row["elapsed_ms"],
        "period_ms": (period_us + 500) // 1000 if period_us > 0 else 0,
        "body_ms": 0,
        "period_us": period_us,
        "body_us": 0,
        "qti_wait_us": 0,
        "policy_us": 0,
        "drive_us": 0,
        "qti_sample_us": 0,
        "raw_left": row["raw_left"],
        "raw_middle": row["raw_middle"],
        "raw_right": row["raw_right"],
        "raw_physical_p7": row.get("raw_physical_p7", 0),
        "raw_physical_p6": row.get("raw_physical_p6", 0),
        "raw_physical_p5": row.get("raw_physical_p5", 0),
        "raw_physical_p4": row.get("raw_physical_p4", 0),
        "encoder_left_ticks": 0,
        "encoder_right_ticks": 0,
        "obs_left_q1000": row["obs_left_q1000"],
        "obs_middle_q1000": row["obs_middle_q1000"],
        "obs_right_q1000": row["obs_right_q1000"],
        "action_left_q1000": row["action_left_q1000"],
        "action_right_q1000": row["action_right_q1000"],
        "command_left_ticks": row["command_left_ticks"],
        "command_right_ticks": row["command_right_ticks"],
        "flags": row["flags"],
        "ramlog_kind": row["ramlog_kind"],
    }


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
        if header["header_size"] < HEADER_SIZE:
            raise SystemExit(
                f"{path}: header size {header['header_size']} < parser base {HEADER_SIZE}"
            )
        extra_size = header["header_size"] - HEADER_SIZE
        extra = f.read(extra_size)
        if len(extra) != extra_size:
            raise SystemExit(f"{path}: short read for {extra_size} header extra bytes")
        if header["header_size"] >= HEADER_SIZE_V9:
            extra_values = struct.unpack(
                HEADER_EXTRA_FMT_V9,
                extra[: struct.calcsize(HEADER_EXTRA_FMT_V9)],
            )
            header.update(dict(zip(HEADER_EXTRA_NAMES_V9, extra_values)))
        elif header["header_size"] >= HEADER_SIZE_V8:
            extra_values = struct.unpack(
                HEADER_EXTRA_FMT_V8,
                extra[: struct.calcsize(HEADER_EXTRA_FMT_V8)],
            )
            header.update(dict(zip(HEADER_EXTRA_NAMES_V8, extra_values)))
            for name in HEADER_EXTRA_NAMES_V9[len(HEADER_EXTRA_NAMES_V8):]:
                header[name] = None
        elif header["header_size"] >= HEADER_SIZE_V2:
            extra_values = struct.unpack(
                HEADER_EXTRA_FMT_V2,
                extra[: struct.calcsize(HEADER_EXTRA_FMT_V2)],
            )
            header.update(dict(zip(HEADER_EXTRA_NAMES_V2, extra_values)))
            for name in HEADER_EXTRA_NAMES_V9[len(HEADER_EXTRA_NAMES_V2):]:
                header[name] = None
        else:
            for name in HEADER_EXTRA_NAMES_V9:
                header[name] = None

        if header["row_size"] == ROW_SIZE:
            row_fmt = ROW_FMT
            row_size = ROW_SIZE
            row_names = ROW_NAMES
        elif header["row_size"] == ROW_SIZE_V2:
            row_fmt = ROW_FMT_V2
            row_size = ROW_SIZE_V2
            row_names = ROW_NAMES_V2
        elif header["row_size"] == ROW_SIZE_V3:
            row_fmt = ROW_FMT_V3
            row_size = ROW_SIZE_V3
            row_names = ROW_NAMES_V3
        elif header["version"] == 7 and header["row_size"] == ROW_SIZE_V7:
            row_fmt = ROW_FMT_V7
            row_size = ROW_SIZE_V7
            row_names = ROW_NAMES_V7
        elif header["version"] in (8, 9) and header["row_size"] == ROW_SIZE_V8:
            row_fmt = ROW_FMT_V8
            row_size = ROW_SIZE_V8
            row_names = ROW_NAMES_V8
        else:
            raise SystemExit(
                f"{path}: row size {header['row_size']} not supported "
                f"(known {ROW_SIZE}, {ROW_SIZE_V2}, {ROW_SIZE_V3}, {ROW_SIZE_V7}, {ROW_SIZE_V8})"
            )

        rows = []
        while True:
            row_bytes = f.read(row_size)
            if not row_bytes:
                break
            if len(row_bytes) != row_size:
                print(f"warning: ignoring partial trailing row of {len(row_bytes)} bytes")
                break
            row = dict(zip(row_names, struct.unpack(row_fmt, row_bytes)))
            if header["version"] in (7, 8, 9):
                row = expand_ramlog_row(row)
            rows.append(row)

    return header, rows


def add_derived(rows):
    prev = None
    for row in rows:
        if "delta_left_ticks_explicit" in row:
            row["delta_left_ticks"] = row["delta_left_ticks_explicit"]
            row["delta_right_ticks"] = row["delta_right_ticks_explicit"]
        elif prev is None:
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
        row["turn_delta_ticks"] = (
            row["command_right_ticks"] - row["command_left_ticks"]
        )
        if row["turn_delta_ticks"] > 2:
            row["turn_sign"] = 1
        elif row["turn_delta_ticks"] < -2:
            row["turn_sign"] = -1
        else:
            row["turn_sign"] = 0
        row["avg_delta_ticks"] = (
            row["delta_left_ticks"] + row["delta_right_ticks"]
        ) / 2.0
        row["left_black"] = int(bool(row["flags"] & 1))
        row["middle_black"] = int(bool(row["flags"] & 2))
        row["right_black"] = int(bool(row["flags"] & 4))
        row["pair_edge"] = int(bool(row["flags"] & (8 | 16)))
        row["one_wheel_floor"] = int(bool(row["flags"] & 32))
        row["both_wheels_floor"] = int(bool(row["flags"] & 64))
        row["commanded_nonzero"] = int(
            row["command_left_ticks"] != 0 or row["command_right_ticks"] != 0
        )
        row["avg_delta_abs_ticks"] = (
            abs(row["delta_left_ticks"]) + abs(row["delta_right_ticks"])
        ) / 2.0
        row["actual_avg_ticks_per_s"] = None
        duration_us = row.get("hold_us") or row.get("period_us") or 0
        if duration_us > 0:
            row["actual_avg_ticks_per_s"] = (
                row["avg_delta_ticks"] * 1000000.0 / duration_us
            )
        prev = row


def range_text(values):
    if not values:
        return "n/a"
    return f"{min(values)}..{max(values)}"


def turn_oscillation_stats(rows):
    turn_rows = [r for r in rows if r.get("turn_sign", 0) != 0]
    if not turn_rows:
        return {
            "turn_rows": 0,
            "sign_flips": 0,
            "flip_intervals_ms": [],
        }

    flips = []
    prev = turn_rows[0]
    for row in turn_rows[1:]:
        if row["turn_sign"] != prev["turn_sign"]:
            dt = row["elapsed_ms"] - prev["elapsed_ms"]
            if dt > 0:
                flips.append(dt)
            prev = row
        elif row["turn_sign"] != 0:
            prev = row

    return {
        "turn_rows": len(turn_rows),
        "sign_flips": len(flips),
        "flip_intervals_ms": flips,
    }


def print_turn_summary(label, rows):
    if not rows:
        print(f"{label}: rows=0")
        return

    deltas = [r["turn_delta_ticks"] for r in rows]
    right_faster = [r for r in rows if r["turn_sign"] > 0]
    left_faster = [r for r in rows if r["turn_sign"] < 0]
    balanced = [r for r in rows if r["turn_sign"] == 0]
    stats = turn_oscillation_stats(rows)
    duration_s = 0.0
    if len(rows) > 1:
        elapsed = rows[-1]["elapsed_ms"] - rows[0]["elapsed_ms"]
        duration_s = max(0.0, elapsed / 1000.0)
    flip_rate = stats["sign_flips"] / duration_s if duration_s > 0.0 else 0.0

    interval_text = "n/a"
    full_period_text = "n/a"
    if stats["flip_intervals_ms"]:
        half_period = mean(stats["flip_intervals_ms"])
        interval_text = (
            f"avg={half_period:.1f} p50={percentile(stats['flip_intervals_ms'], 50)} "
            f"range={range_text(stats['flip_intervals_ms'])}"
        )
        full_period_text = f"{2.0 * half_period:.1f}"

    print(
        f"{label}:",
        f"rows={len(rows)} mean_delta_right_minus_left={mean(deltas):.2f} "
        f"range={range_text(deltas)} "
        f"right_faster={len(right_faster)} "
        f"left_faster={len(left_faster)} balanced={len(balanced)} "
        f"sign_flips={stats['sign_flips']} flip_rate_hz={flip_rate:.2f} "
        f"half_period_ms={interval_text} full_period_ms_est={full_period_text}",
    )


DEFERRED_KIND_NAMES = {
    1: "first",
    2: "last",
    3: "strongest",
    4: "slowest_line",
    10: "both_floor",
    11: "pair_edge",
    12: "command_change",
    13: "periodic",
}


def print_header_summary(header, path):
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
    if header.get("qti_mode") is not None:
        print(
            "firmware_detail:",
            f"qti_mode={header['qti_mode']} "
            f"qti_charge_us={header['qti_charge_us']} "
            f"qti_timeout_us={header['qti_timeout_us']} "
            f"qti_sample_period_us={header['qti_sample_period_us']} "
            f"start_delay_ms={header['start_delay_ms']} "
            f"run_ms={header['run_ms']} "
            f"inference_mode={header['inference_mode']} "
            f"build_id={header['build_id']}",
        )
        if header.get("qti_map_mode") is not None:
            print(
                "firmware_map:",
                f"qti_map_mode={header['qti_map_mode']} "
                f"obs_subtract_min={header['obs_subtract_min']} "
                f"obs_contrast_gain_q1000={header['obs_contrast_gain_q1000']}",
            )
        if header.get("obs_raw_diff") is not None:
            print(
                "firmware_obs_raw_diff:",
                f"obs_raw_diff={header['obs_raw_diff']} "
                f"obs_raw_diff_gain_q1000={header['obs_raw_diff_gain_q1000']} "
                f"obs_raw_diff_span={header['obs_raw_diff_span']}",
            )


def summarize_deferred(header, rows, path):
    print_header_summary(header, path)
    print(
        "sd:",
        f"rows={len(rows)} log_every={header['sd_log_every']} "
        f"max_records={header['sd_max_records']} row_size={header['row_size']} "
        f"version={header['version']} deferred=1",
    )
    stats = {abs(r["loop"]): r for r in rows if r["loop"] < 0}
    events = [r for r in rows if r["loop"] >= 0]

    counts = stats.get(4)
    if counts:
        print(
            "deferred_counts:",
            f"loops={counts['raw_left']} "
            f"all_white={counts['raw_middle']} "
            f"any_black={counts['raw_right']} "
            f"left_black={counts['obs_left_q1000']} "
            f"middle_black={counts['obs_middle_q1000']} "
            f"right_black={counts['obs_right_q1000']} "
            f"pair_edge={counts['period_us']} "
            f"both_floor={counts['command_left_ticks']} "
            f"one_floor={counts['command_right_ticks']} "
            f"command_changes={counts['action_left_q1000']} "
            f"events={counts['action_right_q1000']} "
            f"or_flags={counts['flags']}",
        )

    labels = [(1, "min"), (2, "max"), (3, "avg")]
    for kind, label in labels:
        row = stats.get(kind)
        if not row:
            continue
        print(
            f"deferred_{label}:",
            f"raw=({row['raw_left']},{row['raw_middle']},{row['raw_right']}) "
            f"obs=({row['obs_left_q1000']},{row['obs_middle_q1000']},{row['obs_right_q1000']}) "
            f"actions=({row['action_left_q1000']},{row['action_right_q1000']}) "
            f"cmd=({row['command_left_ticks']},{row['command_right_ticks']}) "
            f"period_us={row.get('period_us', 0)} "
            f"body_us={row.get('body_us', 0)} "
            f"qti_us={row.get('qti_wait_us', 0)} "
            f"policy_us={row.get('policy_us', 0)} "
            f"drive_us={row.get('drive_us', 0)} "
            f"sample_us={row.get('qti_sample_us', 0)}",
        )

    if events:
        kinds = {}
        for row in events:
            kind = row["encoder_left_ticks"]
            kinds[kind] = kinds.get(kind, 0) + 1
        kind_text = ", ".join(
            f"{DEFERRED_KIND_NAMES.get(k, str(k))}={v}" for k, v in sorted(kinds.items())
        )
        print(f"deferred_events: {len(events)} rows {kind_text}")
        for row in events[:20]:
            kind = DEFERRED_KIND_NAMES.get(row["encoder_left_ticks"], str(row["encoder_left_ticks"]))
            print(
                "event:",
                f"kind={kind} loop={row['loop']} elapsed_ms={row['elapsed_ms']} "
                f"raw=({row['raw_left']},{row['raw_middle']},{row['raw_right']}) "
                f"obs=({row['obs_left_q1000']},{row['obs_middle_q1000']},{row['obs_right_q1000']}) "
                f"actions=({row['action_left_q1000']},{row['action_right_q1000']}) "
                f"cmd=({row['command_left_ticks']},{row['command_right_ticks']}) "
                f"flags={row['flags']}",
            )


def summarize(header, rows, path):
    if header["version"] == 5:
        summarize_deferred(header, rows, path)
        return

    add_derived(rows)
    periods = [r["period_ms"] for r in rows[1:] if r["period_ms"] > 0]
    body = [r["body_ms"] for r in rows if r["body_ms"] > 0]
    periods_us = [r["period_us"] for r in rows[1:] if r.get("period_us", 0) > 0]
    body_us = [r["body_us"] for r in rows if r.get("body_us", 0) > 0]
    left_cmd = [r["command_left_ticks"] for r in rows]
    right_cmd = [r["command_right_ticks"] for r in rows]
    avg_cmd = [r["avg_command_ticks"] for r in rows]
    min_ticks = header["min_drive_ticks_per_sec"]
    slow_rows = [r for r in rows if r["avg_command_ticks"] <= min_ticks + 1]
    zero_motion = [
        r for r in rows[1:] if r["delta_left_ticks"] == 0 and r["delta_right_ticks"] == 0
    ]
    pair_rows = [r for r in rows if r.get("pair_edge")]
    pair_floor_rows = [r for r in pair_rows if r.get("one_wheel_floor")]
    pair_both_floor_rows = [r for r in pair_rows if r.get("both_wheels_floor")]
    commanded_zero_motion = [
        r
        for r in rows
        if r.get("commanded_nonzero")
        and r["delta_left_ticks"] == 0
        and r["delta_right_ticks"] == 0
    ]
    diag_rows = "diag_variant" in rows[0] if rows else False

    print_header_summary(header, path)
    print(
        "sd:",
        f"rows={len(rows)} log_every={header['sd_log_every']} "
        f"max_records={header['sd_max_records']} row_size={header['row_size']} "
        f"version={header['version']}",
    )
    if header["version"] in (7, 8, 9):
        print("ramlog: compact in-RAM drive trace; encoder deltas are not recorded")
    if header["version"] in (8, 9):
        print("ramlog_v8: includes raw physical QTI pins P7/P6/P5/P4 before pooling")

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
    if periods_us:
        print(
            "period_us:",
            f"avg={mean(periods_us):.1f} p50={percentile(periods_us, 50)} "
            f"p95={percentile(periods_us, 95)} max={max(periods_us)}",
        )
    if body_us:
        print(
            "body_us:",
            f"avg={mean(body_us):.1f} p50={percentile(body_us, 50)} "
            f"p95={percentile(body_us, 95)} max={max(body_us)}",
        )
        for key in ["qti_wait_us", "policy_us", "drive_us", "qti_sample_us"]:
            values = [r[key] for r in rows if r.get(key, 0) > 0]
            if values:
                print(
                    f"{key}:",
                    f"avg={mean(values):.1f} p50={percentile(values, 50)} "
                    f"p95={percentile(values, 95)} max={max(values)}",
                )

    print(
        "command_ticks:",
        f"left={range_text(left_cmd)} right={range_text(right_cmd)} "
        f"avg={mean(avg_cmd):.2f}",
    )
    print_turn_summary("turn_bias", rows)
    if pair_rows:
        print_turn_summary("pair_edge_turn_bias", pair_rows)
    middle_right_rows = [
        r for r in rows if r.get("middle_black") and r.get("right_black")
    ]
    if middle_right_rows:
        print_turn_summary("middle_right_turn_bias", middle_right_rows)
    middle_left_rows = [
        r for r in rows if r.get("middle_black") and r.get("left_black")
    ]
    if middle_left_rows:
        print_turn_summary("middle_left_turn_bias", middle_left_rows)
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
    if any("raw_physical_p7" in r for r in rows):
        print(
            "raw_qti_physical:",
            f"p7={range_text([r['raw_physical_p7'] for r in rows])} "
            f"p6={range_text([r['raw_physical_p6'] for r in rows])} "
            f"p5={range_text([r['raw_physical_p5'] for r in rows])} "
            f"p4={range_text([r['raw_physical_p4'] for r in rows])}",
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

    if diag_rows:
        print(
            "diag:",
            f"variants={sorted(set(r['diag_variant'] for r in rows))} "
            f"phases={sorted(set(r['diag_phase'] for r in rows))} "
            f"segments={sorted(set(r['segment'] for r in rows if r['segment'] >= 0))}",
        )
        log_us = [r["prev_log_us"] for r in rows if r["prev_log_us"] > 0]
        hold_us = [r["hold_us"] for r in rows if r["hold_us"] > 0]
        if log_us:
            print(
                "prev_log_us:",
                f"avg={mean(log_us):.1f} p50={percentile(log_us, 50)} "
                f"p95={percentile(log_us, 95)} max={max(log_us)}",
            )
        if hold_us:
            print(
                "hold_us:",
                f"avg={mean(hold_us):.1f} p50={percentile(hold_us, 50)} "
                f"p95={percentile(hold_us, 95)} max={max(hold_us)}",
            )
        print(
            "pair_edge_rows:",
            f"{len(pair_rows)}/{len(rows)} ({100.0 * len(pair_rows) / len(rows):.1f}%) "
            f"one_floor={len(pair_floor_rows)} "
            f"both_floor={len(pair_both_floor_rows)}",
        )
        print(
            "commanded_zero_motion:",
            f"{len(commanded_zero_motion)}/{len(rows)} "
            f"({100.0 * len(commanded_zero_motion) / len(rows):.1f}%)",
        )
        for phase in sorted(set(r["diag_phase"] for r in rows)):
            phase_rows = [r for r in rows if r["diag_phase"] == phase]
            if not phase_rows:
                continue
            phase_pair = [r for r in phase_rows if r.get("pair_edge")]
            phase_zero = [
                r
                for r in phase_rows
                if r.get("commanded_nonzero")
                and r["delta_left_ticks"] == 0
                and r["delta_right_ticks"] == 0
            ]
            actual = [
                r["actual_avg_ticks_per_s"]
                for r in phase_rows
                if r.get("actual_avg_ticks_per_s") is not None
            ]
            actual_text = "n/a"
            if actual:
                actual_text = (
                    f"avg_actual_ticks_s={mean(actual):.2f} "
                    f"p50={percentile(actual, 50):.2f}"
                )
            print(
                f"phase_{phase}:",
                f"rows={len(phase_rows)} pair={len(phase_pair)} "
                f"zero_while_commanded={len(phase_zero)} {actual_text}",
            )
        if pair_rows:
            pair_avg_command = [r["avg_command_ticks"] for r in pair_rows]
            pair_avg_delta = [r["avg_delta_ticks"] for r in pair_rows]
            print(
                "pair_edge_command_vs_motion:",
                f"avg_command_ticks={mean(pair_avg_command):.2f} "
                f"avg_delta_ticks={mean(pair_avg_delta):.2f} "
                f"command_range={range_text(pair_avg_command)} "
                f"delta_range={range_text(pair_avg_delta)}",
            )


def write_csv(rows, path):
    add_derived(rows)
    if rows and "diag_variant" in rows[0]:
        base_names = ROW_NAMES_V3
    elif rows and "raw_physical_p7" in rows[0]:
        base_names = ROW_NAMES_V8
    elif rows and "period_us" in rows[0]:
        base_names = ROW_NAMES_V2
    else:
        base_names = ROW_NAMES
    names = base_names + [
        "delta_left_ticks",
        "delta_right_ticks",
        "avg_command_ticks",
        "turn_delta_ticks",
        "turn_sign",
        "avg_delta_ticks",
        "left_black",
        "middle_black",
        "right_black",
        "pair_edge",
        "one_wheel_floor",
        "both_wheels_floor",
        "commanded_nonzero",
        "avg_delta_abs_ticks",
        "actual_avg_ticks_per_s",
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
