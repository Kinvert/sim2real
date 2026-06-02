#!/usr/bin/env python3
import argparse
import collections
import csv
from pathlib import Path


CANDIDATES = [
    ("abs", "abs_left_ticks", "abs_right_ticks"),
    ("rd1", "rd1_left_ticks", "rd1_right_ticks"),
    ("rd2", "rd2_left_ticks", "rd2_right_ticks"),
    ("rd3", "rd3_left_ticks", "rd3_right_ticks"),
    ("agg", "agg_left_ticks", "agg_right_ticks"),
    ("phys3", "phys3_left_ticks", "phys3_right_ticks"),
    ("mir3", "mir3_left_ticks", "mir3_right_ticks"),
    ("r3", "r3_left_ticks", "r3_right_ticks"),
]


def parse_log(path):
    rows = []
    metadata = {}
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip().replace("\x00", "")
            if not line:
                continue
            if line.startswith("checkpoint="):
                metadata["checkpoint"] = line.split("=", 1)[1]
                continue
            if line.startswith("qti_mode="):
                metadata["qti"] = line
                continue
            if line.startswith("compare white="):
                metadata["compare"] = line
                continue
            if not line.startswith("C "):
                continue
            parts = line.split()
            if len(parts) not in (16, 24, 26):
                continue
            if parts[1] == "step":
                continue
            try:
                values = [int(part) for part in parts[1:]]
            except ValueError:
                continue
            row = {
                "step": values[0],
                "source": values[1],
                "raw_left": values[2],
                "raw_middle": values[3],
                "raw_right": values[4],
                "abs_left_ticks": values[5],
                "abs_right_ticks": values[6],
                "rd1_left_ticks": values[7],
                "rd1_right_ticks": values[8],
                "rd2_left_ticks": values[9],
                "rd2_right_ticks": values[10],
                "rd3_left_ticks": values[11],
                "rd3_right_ticks": values[12],
                "agg_left_ticks": values[13],
                "agg_right_ticks": values[14],
            }
            if len(values) in (23, 25):
                row.update(
                    {
                        "phys0": values[15],
                        "phys1": values[16],
                        "phys2": values[17],
                        "phys3": values[18],
                        "phys3_left_ticks": values[19],
                        "phys3_right_ticks": values[20],
                        "mir3_left_ticks": values[21],
                        "mir3_right_ticks": values[22],
                        "r3_left_ticks": values[23] if len(values) == 25 else None,
                        "r3_right_ticks": values[24] if len(values) == 25 else None,
                    }
                )
            else:
                row.update(
                    {
                        "phys0": None,
                        "phys1": None,
                        "phys2": None,
                        "phys3": None,
                        "phys3_left_ticks": None,
                        "phys3_right_ticks": None,
                        "mir3_left_ticks": None,
                        "mir3_right_ticks": None,
                        "r3_left_ticks": None,
                        "r3_right_ticks": None,
                    }
                )
            rows.append(row)
    return metadata, rows


def range_text(values):
    if not values:
        return "n/a"
    return f"{min(values)}..{max(values)}"


def pair_counts(rows, left_key, right_key):
    usable = [row for row in rows if row.get(left_key) is not None and row.get(right_key) is not None]
    counts = collections.Counter((row[left_key], row[right_key]) for row in usable)
    return " ".join(
        f"{left}/{right}:{count}"
        for (left, right), count in sorted(counts.items())
    ) or "n/a"


def both_floor_count(rows, left_key, right_key, min_ticks):
    return sum(
        1 for row in rows
        if row.get(left_key) is not None
        and row.get(right_key) is not None
        if row[left_key] <= min_ticks and row[right_key] <= min_ticks
    )


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--min-ticks", type=int, default=20)
    args = parser.parse_args()

    metadata, rows = parse_log(args.log)
    print(f"candidate_compare: path={args.log} rows={len(rows)}")
    if metadata.get("checkpoint"):
        print(f"checkpoint={metadata['checkpoint']}")
    if metadata.get("qti"):
        print(metadata["qti"])
    if metadata.get("compare"):
        print(metadata["compare"])

    if rows:
        print(
            "raw:",
            f"left={range_text([row['raw_left'] for row in rows])} "
            f"middle={range_text([row['raw_middle'] for row in rows])} "
            f"right={range_text([row['raw_right'] for row in rows])}",
        )
        physical_rows = [row for row in rows if row.get("phys0") is not None]
        if physical_rows:
            print(
                "physical:",
                f"phys0={range_text([row['phys0'] for row in physical_rows])} "
                f"phys1={range_text([row['phys1'] for row in physical_rows])} "
                f"phys2={range_text([row['phys2'] for row in physical_rows])} "
                f"phys3={range_text([row['phys3'] for row in physical_rows])}",
            )
        sources = collections.Counter(row["source"] for row in rows)
        print("sources:", " ".join(f"{key}:{sources[key]}" for key in sorted(sources)))
        for name, left_key, right_key in CANDIDATES:
            both_floor = both_floor_count(rows, left_key, right_key, args.min_ticks)
            print(
                f"{name}:",
                f"pairs={pair_counts(rows, left_key, right_key)}",
                f"both_floor={both_floor}/{len(rows)}",
            )
    if args.csv:
        write_csv(args.csv, rows)


if __name__ == "__main__":
    main()
