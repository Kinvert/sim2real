#!/usr/bin/env python3
import argparse
import configparser
import importlib.util
import statistics
from pathlib import Path


def load_equivalence_module(root):
    path = root / "ocean" / "line_follow" / "scripts" / "line-follow-model-equivalence.py"
    spec = importlib.util.spec_from_file_location("line_follow_model_equivalence", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"could not load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_env_defaults(root):
    config = configparser.ConfigParser()
    config.read(root / "config" / "line_follow.ini")
    env = config["env"] if config.has_section("env") else {}

    def get_float(name, fallback):
        try:
            return float(env.get(name, fallback))
        except (TypeError, ValueError):
            return fallback

    def get_int(name, fallback):
        try:
            return int(round(float(env.get(name, fallback))))
        except (TypeError, ValueError):
            return fallback

    return {
        "max_wheel_speed_mps": get_float("max_wheel_speed_mps", 0.116),
        "command_deadband": get_float("command_deadband", 0.04),
        "min_drive_ticks_per_sec": get_int("min_drive_ticks_per_sec", 6),
    }


def parse_live_rows(path):
    rows = []
    checkpoint = None
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("checkpoint="):
                checkpoint = line.split("=", 1)[1]
                continue
            if not line.startswith("L "):
                continue
            parts = line.split()
            if len(parts) < 21:
                continue
            try:
                rows.append(
                    {
                        "step": int(parts[1]),
                        "raw": [int(parts[2]), int(parts[3]), int(parts[4])],
                        "printed_obs": [int(parts[11]), int(parts[12]), int(parts[13])],
                        "printed_left": int(parts[19]),
                        "printed_right": int(parts[20]),
                    }
                )
            except ValueError:
                continue
    return checkpoint, rows


def round_div(numerator, denominator):
    if denominator <= 0:
        raise ValueError("denominator must be positive")
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def normalize_raw(raw, white, black):
    denom = black - white
    if denom == 0:
        return 1000 if raw >= black else 0
    return clamp(((raw - white) * 1000) // denom, 0, 1000)


def obs_absolute(raw, white, black):
    return [normalize_raw(value, white, black) for value in raw]


def obs_rawdiff(raw, white, black, gain_q1000, span):
    baseline = min(raw)
    if span <= 0:
        span = black - white
    if span <= 0:
        span = 1
    obs = []
    for value in raw:
        diff = value - baseline
        q1000 = round_div(diff * 1000, span)
        if gain_q1000 <= 0 or q1000 <= 0:
            q1000 = 0
        else:
            saturating = (1000 * 1000 + gain_q1000 - 1) // gain_q1000
            if q1000 >= saturating:
                q1000 = 1000
            else:
                q1000 = round_div(q1000 * gain_q1000, 1000)
        obs.append(clamp(q1000, 0, 1000))
    return obs


def range_text(values):
    if not values:
        return "n/a"
    return f"{min(values)}..{max(values)}"


def mean_text(values):
    if not values:
        return "n/a"
    return f"{statistics.fmean(values):.1f}"


def default_candidates():
    return [
        {"name": "absolute", "kind": "absolute"},
        {"name": "rawdiff_gain1", "kind": "rawdiff", "gain": 1000, "span": 0},
        {"name": "rawdiff_gain2", "kind": "rawdiff", "gain": 2000, "span": 0},
        {"name": "rawdiff_gain3", "kind": "rawdiff", "gain": 3000, "span": 0},
    ]


def evaluate_candidate(candidate, rows, model, eq, args):
    stats = {
        "obs": [[], [], []],
        "actions": [[], []],
        "ticks": [[], []],
        "both_floor": 0,
        "one_floor": 0,
        "slow": 0,
    }
    examples = []
    for row in rows:
        raw = row["raw"]
        if candidate["kind"] == "rawdiff":
            obs = obs_rawdiff(
                raw,
                args.qti_white_time,
                args.qti_black_time,
                candidate.get("gain", 1000),
                candidate.get("span", 0),
            )
        else:
            obs = obs_absolute(raw, args.qti_white_time, args.qti_black_time)
        action_fixed, action_q1000 = eq.forward_fixed(model, obs)
        ticks = [
            eq.action_fixed_to_ticks(
                action_fixed[0],
                args.max_wheel_speed_mps,
                args.command_deadband,
                args.min_drive_ticks_per_sec,
            ),
            eq.action_fixed_to_ticks(
                action_fixed[1],
                args.max_wheel_speed_mps,
                args.command_deadband,
                args.min_drive_ticks_per_sec,
            ),
        ]
        for i in range(3):
            stats["obs"][i].append(obs[i])
        for i in range(2):
            stats["actions"][i].append(action_q1000[i])
            stats["ticks"][i].append(ticks[i])
        left_floor = ticks[0] <= args.min_drive_ticks_per_sec
        right_floor = ticks[1] <= args.min_drive_ticks_per_sec
        if left_floor and right_floor:
            stats["both_floor"] += 1
        if left_floor != right_floor:
            stats["one_floor"] += 1
        if (ticks[0] + ticks[1]) / 2.0 <= args.min_drive_ticks_per_sec + 1:
            stats["slow"] += 1
        if len(examples) < args.examples:
            examples.append((row["step"], raw, obs, action_q1000, ticks))
    return stats, examples


def print_candidate(name, stats, examples, row_count):
    left_ticks, right_ticks = stats["ticks"]
    print(
        f"{name}: rows={row_count} "
        f"ticks_left={range_text(left_ticks)} ticks_right={range_text(right_ticks)} "
        f"mean_ticks={mean_text([(l + r) / 2.0 for l, r in zip(left_ticks, right_ticks)])} "
        f"both_floor={stats['both_floor']} one_floor={stats['one_floor']} "
        f"slow={stats['slow']}"
    )
    print(
        "  obs "
        f"left={range_text(stats['obs'][0])} "
        f"middle={range_text(stats['obs'][1])} "
        f"right={range_text(stats['obs'][2])}; "
        "action_q1000 "
        f"left={range_text(stats['actions'][0])} "
        f"right={range_text(stats['actions'][1])}"
    )
    for step, raw, obs, action_q1000, ticks in examples:
        print(
            "  example "
            f"step={step} raw={raw} obs={obs} action_q1000={action_q1000} "
            f"ticks={ticks}"
        )


def main():
    root = Path(__file__).resolve().parents[3]
    defaults = read_env_defaults(root)
    eq = load_equivalence_module(root)
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--hidden-size", type=int, default=8)
    parser.add_argument("--num-layers", type=int, default=0)
    parser.add_argument("--qti-white-time", type=int, default=80)
    parser.add_argument("--qti-black-time", type=int, default=350)
    parser.add_argument("--max-wheel-speed-mps", type=float, default=defaults["max_wheel_speed_mps"])
    parser.add_argument("--command-deadband", type=float, default=defaults["command_deadband"])
    parser.add_argument("--min-drive-ticks-per-sec", type=int, default=defaults["min_drive_ticks_per_sec"])
    parser.add_argument("--examples", type=int, default=1)
    args = parser.parse_args()

    parsed_logs = []
    checkpoint_from_log = None
    for path in args.logs:
        checkpoint, rows = parse_live_rows(path)
        if checkpoint and checkpoint_from_log is None:
            checkpoint_from_log = Path(checkpoint)
        if not rows:
            raise SystemExit(f"{path}: no live L rows found")
        parsed_logs.append((path, rows))

    checkpoint = args.checkpoint or checkpoint_from_log or eq.latest_checkpoint(root)
    if checkpoint is None:
        raise SystemExit("No checkpoint found")
    checkpoint = checkpoint.resolve()
    model = eq.load_model(checkpoint, args.hidden_size, args.num_layers)

    print(f"checkpoint={checkpoint}")
    print(
        f"qti_white={args.qti_white_time} qti_black={args.qti_black_time} "
        f"max_speed={args.max_wheel_speed_mps} deadband={args.command_deadband} "
        f"min_ticks={args.min_drive_ticks_per_sec}"
    )
    for path, rows in parsed_logs:
        print(f"\nlog={path} rows={len(rows)} raw_left={range_text([r['raw'][0] for r in rows])} "
              f"raw_middle={range_text([r['raw'][1] for r in rows])} "
              f"raw_right={range_text([r['raw'][2] for r in rows])}")
        for candidate in default_candidates():
            stats, examples = evaluate_candidate(candidate, rows, model, eq, args)
            print_candidate(candidate["name"], stats, examples, len(rows))


if __name__ == "__main__":
    main()
