#!/usr/bin/env python3
import argparse
import configparser
import csv
import math
import random
import struct
from pathlib import Path


OBS_SIZE = 3
NUM_ACTIONS = 2
DECODER_SIZE = NUM_ACTIONS + 1
FIXED_SCALE = 16384
TIRE_DIAMETER_M = 0.065
TICKS_PER_REV = 64.0


CANNED_CASES = [
    ("all_white", [0, 0, 0]),
    ("all_black", [1000, 1000, 1000]),
    ("left", [1000, 0, 0]),
    ("middle", [0, 1000, 0]),
    ("right", [0, 0, 1000]),
    ("soft_left", [800, 200, 0]),
    ("soft_right", [0, 200, 800]),
    ("balanced_edge", [200, 1000, 200]),
]


def align4(index):
    return (index + 3) & ~3


def round_div(numerator, denominator):
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def float_to_q1000(value):
    if value >= 0.0:
        return int(value * 1000.0 + 0.5)
    return -int((-value) * 1000.0 + 0.5)


def latest_file(paths):
    paths = [path for path in paths if path.is_file()]
    if not paths:
        return None
    return max(paths, key=lambda path: path.stat().st_mtime)


def latest_checkpoint(root):
    return latest_file((root / "checkpoints" / "line_follow").glob("*/*.bin"))


def default_robot_csv(root):
    build = root / "ocean" / "line_follow" / "build" / "line-follow"
    return latest_file(list(build.glob("lf*_*.csv")) + list(build.glob("lf_log_*.csv")))


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


def load_floats(path):
    data = path.read_bytes()
    if len(data) % 4 != 0:
        raise SystemExit(f"{path}: size is not a multiple of 4 bytes")
    return list(struct.unpack("<" + "f" * (len(data) // 4), data))


def expected_raw_float_count(hidden_size, num_layers):
    index = 0
    index += hidden_size * OBS_SIZE
    index = align4(index)
    index += DECODER_SIZE * hidden_size
    index = align4(index)
    index += NUM_ACTIONS
    if num_layers > 0:
        index = align4(index)
        for _ in range(num_layers):
            index += 3 * hidden_size * hidden_size
            index = align4(index)
    return index


def take_aligned(floats, index, count):
    end = index + count
    if end > len(floats):
        raise SystemExit(
            f"checkpoint ended while reading {count} floats at aligned index {index}"
        )
    return floats[index:end], align4(end)


def load_feedforward_model(path, hidden_size, num_layers):
    if num_layers != 0:
        raise SystemExit("folded equivalence is only defined for num_layers=0")

    raw = load_floats(path)
    expected = expected_raw_float_count(hidden_size, num_layers)
    if len(raw) != expected:
        raise SystemExit(
            f"{path}: raw_floats={len(raw)} expected={expected} "
            f"hidden_size={hidden_size} num_layers={num_layers}"
        )

    padded = raw + [0.0] * 7
    index = 0
    encoder, index = take_aligned(padded, index, hidden_size * OBS_SIZE)
    decoder, index = take_aligned(padded, index, DECODER_SIZE * hidden_size)
    _log_std, _index = take_aligned(padded, index, NUM_ACTIONS)

    folded = []
    for action in range(NUM_ACTIONS):
        for obs in range(OBS_SIZE):
            total = 0.0
            for hidden in range(hidden_size):
                total += (
                    decoder[action * hidden_size + hidden]
                    * encoder[hidden * OBS_SIZE + obs]
                )
            folded.append(total)
    folded_q = [round(value * FIXED_SCALE) for value in folded]
    return {
        "raw_floats": len(raw),
        "encoder": encoder,
        "decoder": decoder,
        "folded": folded,
        "folded_q": folded_q,
        "hidden_size": hidden_size,
    }


def forward_full(model, obs):
    hidden_size = model["hidden_size"]
    hidden = []
    for h in range(hidden_size):
        total = 0.0
        for i in range(OBS_SIZE):
            total += model["encoder"][h * OBS_SIZE + i] * obs[i]
        hidden.append(total)

    actions = []
    for action in range(NUM_ACTIONS):
        total = 0.0
        for h in range(hidden_size):
            total += model["decoder"][action * hidden_size + h] * hidden[h]
        actions.append(clamp(total, -1.0, 1.0))
    return actions


def forward_folded(model, obs):
    actions = []
    for action in range(NUM_ACTIONS):
        total = 0.0
        for i in range(OBS_SIZE):
            total += model["folded"][action * OBS_SIZE + i] * obs[i]
        actions.append(clamp(total, -1.0, 1.0))
    return actions


def forward_fixed(model, obs_q1000):
    action_fixed = []
    action_q1000 = []
    for action in range(NUM_ACTIONS):
        total = 0
        for i in range(OBS_SIZE):
            total += obs_q1000[i] * model["folded_q"][action * OBS_SIZE + i]
        fixed = clamp(round_div(total, 1000), -FIXED_SCALE, FIXED_SCALE)
        action_fixed.append(fixed)
        action_q1000.append(clamp(round_div(fixed * 1000, FIXED_SCALE), -1000, 1000))
    return action_fixed, action_q1000


def action_to_ticks(action, max_wheel_speed_mps, command_deadband, min_ticks):
    unit = clamp(action, -1.0, 1.0)
    if unit < 0.0:
        unit = 0.0
    if unit < command_deadband:
        unit = 0.0

    tick_scale = (
        max_wheel_speed_mps / (math.pi * TIRE_DIAMETER_M)
    ) * TICKS_PER_REV
    max_ticks = int(tick_scale + 0.5)
    min_ticks = clamp(min_ticks, 0, max_ticks)
    ticks = unit * tick_scale
    rounded = int(ticks + 0.5) if ticks >= 0.0 else int(ticks - 0.5)
    if rounded < min_ticks:
        return min_ticks
    return rounded


def action_q1000_to_ticks(action_q1000, max_wheel_speed_mps, command_deadband, min_ticks):
    unit = clamp(action_q1000, -1000, 1000)
    if unit < 0:
        unit = 0
    deadband_q1000 = int(command_deadband * 1000.0 + 0.5)
    if unit < deadband_q1000:
        unit = 0

    tick_scale = (
        max_wheel_speed_mps / (math.pi * TIRE_DIAMETER_M)
    ) * TICKS_PER_REV
    tick_scale_q1000 = int(tick_scale * 1000.0 + 0.5)
    max_ticks = int(tick_scale + 0.5)
    min_ticks = clamp(min_ticks, 0, max_ticks)
    rounded = round_div(unit * tick_scale_q1000, 1000 * 1000)
    if rounded < min_ticks:
        return min_ticks
    return rounded


def action_fixed_to_ticks(action_fixed, max_wheel_speed_mps, command_deadband, min_ticks):
    unit = clamp(action_fixed, -FIXED_SCALE, FIXED_SCALE)
    if unit < 0:
        unit = 0
    deadband_fixed = int(command_deadband * FIXED_SCALE + 0.5)
    if unit < deadband_fixed:
        unit = 0

    tick_scale = (
        max_wheel_speed_mps / (math.pi * TIRE_DIAMETER_M)
    ) * TICKS_PER_REV
    tick_scale_q1000 = int(tick_scale * 1000.0 + 0.5)
    max_ticks = int(tick_scale + 0.5)
    min_ticks = clamp(min_ticks, 0, max_ticks)
    rounded = round_div(unit * tick_scale_q1000, FIXED_SCALE * 1000)
    if rounded < min_ticks:
        return min_ticks
    return rounded


def canned_samples():
    for name, obs_q1000 in CANNED_CASES:
        yield name, [value / 1000.0 for value in obs_q1000], obs_q1000


def random_samples(count, seed):
    rng = random.Random(seed)
    for i in range(count):
        obs = [rng.random() for _ in range(OBS_SIZE)]
        obs_q1000 = [clamp(int(value * 1000.0 + 0.5), 0, 1000) for value in obs]
        yield f"random_{i}", obs, obs_q1000


def robot_samples(path):
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            obs_q1000 = [
                int(row["obs_left_q1000"]),
                int(row["obs_middle_q1000"]),
                int(row["obs_right_q1000"]),
            ]
            yield f"{path.name}:{idx}", [value / 1000.0 for value in obs_q1000], obs_q1000


def compare_dataset(name, samples, model, args):
    count = 0
    max_full_fold_action_diff = 0.0
    max_fixed_fold_qobs_action_q1000_diff = 0
    full_fold_tick_mismatches = 0
    fixed_fold_qobs_tick_mismatches = 0
    fixed_full_tick_mismatches = 0
    examples = []

    for sample_name, obs, obs_q1000 in samples:
        count += 1
        full = forward_full(model, obs)
        folded = forward_folded(model, obs)
        obs_from_q = [value / 1000.0 for value in obs_q1000]
        folded_qobs = forward_folded(model, obs_from_q)
        fixed_action, fixed_q1000 = forward_fixed(model, obs_q1000)
        fixed = [value / 1000.0 for value in fixed_q1000]

        full_ticks = [
            action_to_ticks(
                value,
                args.max_wheel_speed_mps,
                args.command_deadband,
                args.min_drive_ticks_per_sec,
            )
            for value in full
        ]
        folded_ticks = [
            action_to_ticks(
                value,
                args.max_wheel_speed_mps,
                args.command_deadband,
                args.min_drive_ticks_per_sec,
            )
            for value in folded
        ]
        folded_qobs_ticks = [
            action_to_ticks(
                value,
                args.max_wheel_speed_mps,
                args.command_deadband,
                args.min_drive_ticks_per_sec,
            )
            for value in folded_qobs
        ]
        fixed_ticks = [
            action_fixed_to_ticks(
                value,
                args.max_wheel_speed_mps,
                args.command_deadband,
                args.min_drive_ticks_per_sec,
            )
            for value in fixed_action
        ]

        for a in range(NUM_ACTIONS):
            max_full_fold_action_diff = max(
                max_full_fold_action_diff,
                abs(full[a] - folded[a]),
            )
            folded_q1000 = clamp(float_to_q1000(folded_qobs[a]), -1000, 1000)
            max_fixed_fold_qobs_action_q1000_diff = max(
                max_fixed_fold_qobs_action_q1000_diff,
                abs(fixed_q1000[a] - folded_q1000),
            )

        if full_ticks != folded_ticks:
            full_fold_tick_mismatches += 1
        if fixed_ticks != folded_qobs_ticks:
            fixed_fold_qobs_tick_mismatches += 1
            if len(examples) < 5:
                examples.append(
                    (
                        sample_name,
                        obs_q1000,
                        [round(v, 6) for v in folded_qobs],
                        fixed,
                        folded_qobs_ticks,
                        fixed_ticks,
                    )
                )
        if fixed_ticks != full_ticks:
            fixed_full_tick_mismatches += 1

    fixed_mismatch_frac = (
        fixed_fold_qobs_tick_mismatches / count if count > 0 else 0.0
    )
    ok = (
        max_full_fold_action_diff <= args.max_fold_action_diff
        and full_fold_tick_mismatches == 0
        and fixed_mismatch_frac <= args.max_fixed_tick_mismatch_frac
    )

    print(f"\n{name}:")
    print(f"  samples={count}")
    print(f"  full_vs_folded max_action_diff={max_full_fold_action_diff:.9g}")
    print(f"  full_vs_folded tick_mismatches={full_fold_tick_mismatches}")
    print(
        "  fixed_vs_folded_qobs "
        f"max_action_q1000_diff={max_fixed_fold_qobs_action_q1000_diff} "
        f"tick_mismatches={fixed_fold_qobs_tick_mismatches} "
        f"({100.0 * fixed_mismatch_frac:.3f}%)"
    )
    print(f"  fixed_vs_full tick_mismatches={fixed_full_tick_mismatches}")
    for example in examples:
        sample_name, obs_q1000, folded_qobs, fixed, folded_ticks, fixed_ticks = example
        print(
            "  mismatch "
            f"{sample_name} obs_q1000={obs_q1000} "
            f"folded_qobs={folded_qobs} fixed={fixed} "
            f"folded_ticks={folded_ticks} fixed_ticks={fixed_ticks}"
        )
    return ok


def main():
    root = Path(__file__).resolve().parents[3]
    defaults = read_env_defaults(root)

    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", nargs="?", type=Path)
    parser.add_argument("--robot-csv", action="append", type=Path)
    parser.add_argument("--hidden-size", type=int, default=8)
    parser.add_argument("--num-layers", type=int, default=0)
    parser.add_argument("--random-count", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=12345)
    parser.add_argument(
        "--max-wheel-speed-mps",
        type=float,
        default=defaults["max_wheel_speed_mps"],
    )
    parser.add_argument(
        "--command-deadband",
        type=float,
        default=defaults["command_deadband"],
    )
    parser.add_argument(
        "--min-drive-ticks-per-sec",
        type=int,
        default=defaults["min_drive_ticks_per_sec"],
    )
    parser.add_argument("--max-fold-action-diff", type=float, default=2e-5)
    parser.add_argument("--max-fixed-tick-mismatch-frac", type=float, default=0.01)
    args = parser.parse_args()

    checkpoint = args.checkpoint or latest_checkpoint(root)
    if checkpoint is None:
        raise SystemExit("No line_follow checkpoint found")
    checkpoint = checkpoint.resolve()
    model = load_feedforward_model(checkpoint, args.hidden_size, args.num_layers)

    robot_csvs = args.robot_csv
    if robot_csvs is None:
        latest_csv = default_robot_csv(root)
        robot_csvs = [latest_csv] if latest_csv is not None else []

    print(f"checkpoint={checkpoint}")
    print(
        f"shape: obs={OBS_SIZE} hidden={args.hidden_size} "
        f"layers={args.num_layers} raw_floats={model['raw_floats']}"
    )
    print(
        f"fixed_scale={FIXED_SCALE} max_speed={args.max_wheel_speed_mps} "
        f"deadband={args.command_deadband} min_ticks={args.min_drive_ticks_per_sec}"
    )

    results = [
        compare_dataset("canned", canned_samples(), model, args),
        compare_dataset(
            "random",
            random_samples(args.random_count, args.seed),
            model,
            args,
        ),
    ]
    for path in robot_csvs:
        results.append(compare_dataset(f"robot_csv {path}", robot_samples(path), model, args))

    if not all(results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
