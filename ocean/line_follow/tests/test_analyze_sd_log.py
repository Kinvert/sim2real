#!/usr/bin/env python3
import csv
import subprocess
import struct
import sys
from pathlib import Path


MAGIC = 0x4453464C
HEADER_FMT = "<I18i128s"
HEADER_EXTRA_FMT_V8 = "<12i"
HEADER_EXTRA_FMT_V9 = "<16i"
ROW_FMT_V8 = "<19h"


def write_ramlog(path, version):
    extra_fmt = HEADER_EXTRA_FMT_V9 if version == 9 else HEADER_EXTRA_FMT_V8
    header_size = struct.calcsize(HEADER_FMT) + struct.calcsize(extra_fmt)
    row_size = struct.calcsize(ROW_FMT_V8)
    checkpoint = b"synthetic.bin"
    checkpoint += b"\0" * (128 - len(checkpoint))

    header = [
        MAGIC,
        version,
        header_size,
        row_size,
        3,
        3,
        8,
        0,
        80_000_000,
        62,
        110,
        500,
        298,
        13,
        40,
        0,
        16,
        40,
        50,
        checkpoint,
    ]
    extra = [
        2,      # qti_mode: PASM
        1000,
        2000,
        0,
        8000,
        30000,
        2,      # inference_mode: fixed
        12345,
        1,      # qti_map_mode: adjacent max
        0,      # obs_subtract_min
        1000,
        0,
    ]
    if version == 9:
        extra += [
            1,      # obs_raw_diff
            3000,
            0,
            0,
        ]
    rows = [
        [1, 10, 22000, 62, 62, 83, 62, 62, 59, 83, 0, 0, 437, 1000, -1000, 93, 13, 132, 1],
        [2, 32, 22000, 62, 83, 83, 62, 83, 83, 62, 0, 437, 437, 100, -1000, 13, 13, 68, 5],
    ]

    with path.open("wb") as f:
        f.write(struct.pack(HEADER_FMT, *header))
        f.write(struct.pack(extra_fmt, *extra))
        for row in rows:
            f.write(struct.pack(ROW_FMT_V8, *row))


def main():
    root = Path(__file__).resolve().parents[3]
    out_dir = root / "ocean" / "line_follow" / "build" / "tests"
    out_dir.mkdir(parents=True, exist_ok=True)
    analyzer = root / "ocean" / "line_follow" / "scripts" / "analyze-line-follow-sd-log.py"

    for version in (8, 9):
        log_path = out_dir / f"ramlog-v{version}-synthetic.bin"
        csv_path = out_dir / f"ramlog-v{version}-synthetic.csv"
        write_ramlog(log_path, version)
        result = subprocess.run(
            [sys.executable, str(analyzer), str(log_path), "--csv", str(csv_path)],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        )
        stdout = result.stdout
        required = [
            f"version={version}",
            "ramlog_v8: includes raw physical QTI pins P7/P6/P5/P4 before pooling",
            "firmware_map: qti_map_mode=1 obs_subtract_min=0 obs_contrast_gain_q1000=1000",
            "raw_qti_physical: p7=62..62 p6=62..83 p5=59..83 p4=62..83",
            "turn_bias: rows=2 mean_delta_right_minus_left=-40.00",
        ]
        if version == 9:
            required.append(
                "firmware_obs_raw_diff: obs_raw_diff=1 obs_raw_diff_gain_q1000=3000 obs_raw_diff_span=0"
            )
        for text in required:
            if text not in stdout:
                raise AssertionError(f"missing analyzer output: {text}\n{stdout}")

        with csv_path.open(newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if len(rows) != 2:
            raise AssertionError(f"expected 2 csv rows, got {len(rows)}")
        first = rows[0]
        checks = {
            "raw_physical_p7": "62",
            "raw_physical_p6": "62",
            "raw_physical_p5": "59",
            "raw_physical_p4": "83",
            "command_left_ticks": "93",
            "command_right_ticks": "13",
            "turn_delta_ticks": "-80",
            "turn_sign": "-1",
        }
        for key, expected in checks.items():
            actual = first.get(key)
            if actual != expected:
                raise AssertionError(f"{key}: got {actual}, expected {expected}")

    print("analyze-line-follow-sd-log v8/v9 test passed")


if __name__ == "__main__":
    main()
