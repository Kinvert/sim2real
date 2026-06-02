#!/usr/bin/env python3
import csv
import subprocess
import sys
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[3]
    out_dir = root / "ocean" / "line_follow" / "build" / "tests"
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "synthetic-candidate-compare.txt"
    csv_path = out_dir / "synthetic-candidate-compare.csv"
    log_path.write_text(
        "\n".join(
            [
                "LINE_FOLLOW_CANDIDATE_COMPARE v1",
                "checkpoint=/tmp/checkpoint.bin",
                "qti_mode=2 qti_map_mode=1 charge_us=1000 timeout_us=1000 sample_period_us=0",
                "compare white=80 black=350 agg_white=62 agg_black=110 min_ticks=20 max_speed_mm_s=260",
                "C step src raw0 raw1 raw2 absL absR rd1L rd1R rd2L rd2R rd3L rd3R aggL aggR",
                "C 0 0 62 62 82 20 20 33 20 66 20 81 20 81 20",
                "C 1 0 63 63 82 20 20 31 20 62 20 81 20 81 20",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    analyzer = (
        root
        / "ocean"
        / "line_follow"
        / "scripts"
        / "analyze-line-follow-candidate-compare.py"
    )
    result = subprocess.run(
        [sys.executable, str(analyzer), str(log_path), "--csv", str(csv_path)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    stdout = result.stdout
    required = [
        "candidate_compare:",
        "rows=2",
        "raw: left=62..63 middle=62..63 right=82..82",
        "abs: pairs=20/20:2 both_floor=2/2",
        "rd3: pairs=81/20:2 both_floor=0/2",
    ]
    for text in required:
        if text not in stdout:
            raise AssertionError(f"missing analyzer output: {text}\n{stdout}")

    with csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 2:
        raise AssertionError(f"expected 2 rows, got {len(rows)}")
    if rows[0]["rd3_left_ticks"] != "81" or rows[0]["agg_left_ticks"] != "81":
        raise AssertionError(f"unexpected csv row: {rows[0]}")
    print("analyze-line-follow-candidate-compare test passed")


if __name__ == "__main__":
    main()
