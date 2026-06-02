#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def normalize_q1000(raw, white, black):
    denom = black - white
    if denom == 0:
        return 1000 if raw >= black else 0
    return clamp(((raw - white) * 1000) // denom, 0, 1000)


def round_div(numerator, denominator):
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def rawdiff_q1000(raw3, white, black, gain_q1000, span):
    baseline = min(raw3)
    if span <= 0:
        span = black - white
    if span <= 0:
        span = 1
    out = []
    for raw in raw3:
        value = round_div((raw - baseline) * 1000, span)
        if value <= 0 or gain_q1000 <= 0:
            value = 0
        else:
            value = round_div(value * gain_q1000, 1000)
        out.append(clamp(value, 0, 1000))
    return out


def raw4_from_row(row, source):
    prefix = "grp" if source == "grouped" else "rc"
    return [int(row[f"{prefix}{pin}"]) for pin in (7, 6, 5, 4)]


def raw3_from_raw4(raw4, qti_map_mode):
    if qti_map_mode == "three-pin":
        return raw4[:3]
    if qti_map_mode == "p654":
        return raw4[1:4]
    if qti_map_mode == "adjacent-max":
        return [
            max(raw4[0], raw4[1]),
            max(raw4[1], raw4[2]),
            max(raw4[2], raw4[3]),
        ]
    raise ValueError(qti_map_mode)


def obs_from_raw3(raw3, obs_mode, white, black, rawdiff_gain_q1000, rawdiff_span):
    if obs_mode == "normalized":
        return [normalize_q1000(raw, white, black) for raw in raw3]
    if obs_mode == "rawdiff":
        return rawdiff_q1000(raw3, white, black, rawdiff_gain_q1000, rawdiff_span)
    raise ValueError(obs_mode)


def c_name(index, row):
    loop = row.get("loop", str(index))
    elapsed = row.get("host_elapsed_s", "")
    if elapsed:
        try:
            elapsed_ms = int(round(float(elapsed) * 1000.0))
            return f"r{index:03d}_loop{int(float(loop)):04d}_t{elapsed_ms}ms"
        except ValueError:
            pass
    return f"r{index:03d}_loop{loop}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_csv", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--source", choices=["grouped", "rc"], default="grouped")
    parser.add_argument("--qti-map-mode", choices=["three-pin", "p654", "adjacent-max"], default="three-pin")
    parser.add_argument("--obs-mode", choices=["normalized", "rawdiff"], default="normalized")
    parser.add_argument("--white", type=int, default=80)
    parser.add_argument("--black", type=int, default=403)
    parser.add_argument("--rawdiff-gain-q1000", type=int, default=1000)
    parser.add_argument("--rawdiff-span", type=int, default=0)
    parser.add_argument("--limit", type=int, default=160)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--start-row", type=int, default=0)
    args = parser.parse_args()

    if args.limit <= 0:
        raise SystemExit("--limit must be positive")
    if args.stride <= 0:
        raise SystemExit("--stride must be positive")

    cases = []
    with args.capture_csv.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for source_index, row in enumerate(reader):
            if source_index < args.start_row:
                continue
            if source_index % args.stride != 0:
                continue
            raw4 = raw4_from_row(row, args.source)
            raw3 = raw3_from_raw4(raw4, args.qti_map_mode)
            obs = obs_from_raw3(
                raw3,
                args.obs_mode,
                args.white,
                args.black,
                args.rawdiff_gain_q1000,
                args.rawdiff_span,
            )
            cases.append((c_name(len(cases), row), obs))
            if len(cases) >= args.limit:
                break

    if not cases:
        raise SystemExit(f"{args.capture_csv}: no rows selected")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="ascii") as f:
        f.write("/* Generated replay cases. Do not edit by hand. */\n")
        f.write("static FakeCase cases[] = {\n")
        for name, obs in cases:
            f.write(f'  {{"{name}", {{{obs[0]}, {obs[1]}, {obs[2]}}}}},\n')
        f.write("};\n")

    print(
        f"wrote {len(cases)} replay cases to {args.output} "
        f"source={args.source} qti_map_mode={args.qti_map_mode} "
        f"obs_mode={args.obs_mode} start_row={args.start_row}"
    )


if __name__ == "__main__":
    main()
