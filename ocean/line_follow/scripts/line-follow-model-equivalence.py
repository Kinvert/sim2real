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


def align8(index):
    return (index + 7) & ~7


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


def compact_float_count(hidden_size, num_layers):
    return (
        hidden_size * OBS_SIZE
        + DECODER_SIZE * hidden_size
        + NUM_ACTIONS
        + num_layers * 3 * hidden_size * hidden_size
    )


def storage_float_count(hidden_size, num_layers):
    index = 0
    index = align8(index)
    index += hidden_size * OBS_SIZE
    index = align8(index)
    index += DECODER_SIZE * hidden_size
    index = align8(index)
    index += NUM_ACTIONS
    for _ in range(num_layers):
        index = align8(index)
        index += 3 * hidden_size * hidden_size
    return index


def aligned_read_float_count(hidden_size, num_layers):
    return align8(storage_float_count(hidden_size, num_layers))


def checkpoint_to_aligned(values, hidden_size, num_layers):
    storage_count = storage_float_count(hidden_size, num_layers)
    aligned_count = aligned_read_float_count(hidden_size, num_layers)
    if len(values) not in (storage_count, aligned_count):
        raise SystemExit(
            f"checkpoint has {len(values)} floats; expected native storage "
            f"{storage_count} or aligned read {aligned_count}"
        )
    return values + [0.0] * (aligned_count - len(values))


def take_aligned(floats, index, count):
    end = index + count
    if end > len(floats):
        raise SystemExit(
            f"checkpoint ended while reading {count} floats at aligned index {index}"
        )
    return floats[index:end], align8(end)


def load_model(path, hidden_size, num_layers):
    raw = load_floats(path)
    expected = storage_float_count(hidden_size, num_layers)
    aligned_read = aligned_read_float_count(hidden_size, num_layers)
    compact = compact_float_count(hidden_size, num_layers)
    if len(raw) not in (expected, aligned_read):
        raise SystemExit(
            f"{path}: raw_floats={len(raw)} expected_native={expected} "
            f"aligned_read={aligned_read} compact={compact} "
            f"hidden_size={hidden_size} num_layers={num_layers}. "
            "Retrain/export this architecture with padded native checkpoint support."
        )

    padded = checkpoint_to_aligned(raw, hidden_size, num_layers)
    if len(padded) != aligned_read:
        raise SystemExit(
            f"internal alignment error: padded={len(padded)} aligned_read={aligned_read}"
        )
    index = 0
    encoder, index = take_aligned(padded, index, hidden_size * OBS_SIZE)
    decoder, index = take_aligned(padded, index, DECODER_SIZE * hidden_size)
    _log_std, _index = take_aligned(padded, index, NUM_ACTIONS)
    mingru = []
    index = _index
    for _ in range(num_layers):
        weights, index = take_aligned(padded, index, 3 * hidden_size * hidden_size)
        mingru.append(weights)

    folded = None
    folded_q = None
    if num_layers == 0:
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
        "mingru": mingru,
        "folded": folded,
        "folded_q": folded_q,
        "weights_q": [round(value * FIXED_SCALE) for value in padded],
        "hidden_size": hidden_size,
        "num_layers": num_layers,
    }


def sigmoid(x):
    return 1.0 / (1.0 + math.exp(-x))


def forward_full(model, obs, state=None):
    hidden_size = model["hidden_size"]
    num_layers = model["num_layers"]
    if state is None:
        state = [0.0] * hidden_size

    hidden = []
    for h in range(hidden_size):
        total = 0.0
        for i in range(OBS_SIZE):
            total += model["encoder"][h * OBS_SIZE + i] * obs[i]
        hidden.append(total)

    if num_layers > 0:
        combined = []
        mingru_w = model["mingru"][0]
        for o in range(3 * hidden_size):
            total = 0.0
            for h in range(hidden_size):
                total += mingru_w[o * hidden_size + h] * hidden[h]
            combined.append(total)
        mingru_out = []
        for h in range(hidden_size):
            hidden_v = combined[h]
            gate = combined[hidden_size + h]
            highway = combined[2 * hidden_size + h]
            gate_s = sigmoid(gate)
            hidden_tilde = hidden_v + 0.5 if hidden_v >= 0.0 else sigmoid(hidden_v)
            next_state = state[h] + gate_s * (hidden_tilde - state[h])
            highway_s = sigmoid(highway)
            mingru_out.append(highway_s * next_state + (1.0 - highway_s) * hidden[h])
            state[h] = next_state
        hidden = mingru_out

    actions = []
    for action in range(NUM_ACTIONS):
        total = 0.0
        for h in range(hidden_size):
            total += model["decoder"][action * hidden_size + h] * hidden[h]
        actions.append(clamp(total, -1.0, 1.0))
    return actions


def forward_folded(model, obs):
    if model["folded"] is None:
        raise RuntimeError("folded inference is only defined for num_layers=0")
    actions = []
    for action in range(NUM_ACTIONS):
        total = 0.0
        for i in range(OBS_SIZE):
            total += model["folded"][action * OBS_SIZE + i] * obs[i]
        actions.append(clamp(total, -1.0, 1.0))
    return actions


SIGMOID_LUT_Q = [
    5, 6, 7, 8, 9, 10, 12, 13,
    15, 17, 19, 22, 25, 28, 32, 36,
    41, 46, 52, 59, 67, 76, 86, 97,
    110, 124, 141, 159, 180, 204, 230, 261,
    295, 333, 376, 425, 480, 542, 612, 690,
    777, 875, 984, 1107, 1243, 1394, 1562, 1748,
    1953, 2178, 2426, 2695, 2989, 3307, 3649, 4015,
    4406, 4820, 5256, 5712, 6186, 6674, 7173, 7681,
    8192, 8703, 9211, 9710, 10198, 10672, 11128, 11564,
    11978, 12369, 12735, 13077, 13395, 13689, 13958, 14206,
    14431, 14636, 14822, 14990, 15141, 15277, 15400, 15509,
    15607, 15694, 15772, 15842, 15904, 15959, 16008, 16051,
    16089, 16123, 16154, 16180, 16204, 16225, 16243, 16260,
    16274, 16287, 16298, 16308, 16317, 16325, 16332, 16338,
    16343, 16348, 16352, 16356, 16359, 16362, 16365, 16367,
    16369, 16371, 16372, 16374, 16375, 16376, 16377, 16378,
    16379,
]


def sigmoid_q(value):
    lo = -8 * FIXED_SCALE
    hi = 8 * FIXED_SCALE
    if value <= lo:
        return SIGMOID_LUT_Q[0]
    if value >= hi:
        return SIGMOID_LUT_Q[-1]
    step = FIXED_SCALE // 8
    shifted = value - lo
    idx = shifted // step
    frac = shifted - idx * step
    return SIGMOID_LUT_Q[idx] + round_div(
        (SIGMOID_LUT_Q[idx + 1] - SIGMOID_LUT_Q[idx]) * frac, step
    )


def aligned_weight_slices_q(model):
    hidden_size = model["hidden_size"]
    index = 0
    weights_q = model["weights_q"]
    encoder = weights_q[index : index + hidden_size * OBS_SIZE]
    index = align8(index + hidden_size * OBS_SIZE)
    decoder = weights_q[index : index + DECODER_SIZE * hidden_size]
    index = align8(index + DECODER_SIZE * hidden_size)
    index = align8(index + NUM_ACTIONS)
    mingru = []
    for _ in range(model["num_layers"]):
        weights = weights_q[index : index + 3 * hidden_size * hidden_size]
        index = align8(index + 3 * hidden_size * hidden_size)
        mingru.append(weights)
    return encoder, decoder, mingru


def forward_fixed(model, obs_q1000, state_q=None):
    action_fixed = []
    action_q1000 = []
    hidden_size = model["hidden_size"]
    num_layers = model["num_layers"]
    internal_limit = 48 * FIXED_SCALE

    if num_layers == 0 and model["folded_q"] is not None:
        for action in range(NUM_ACTIONS):
            total = 0
            for i in range(OBS_SIZE):
                total += obs_q1000[i] * model["folded_q"][action * OBS_SIZE + i]
            fixed = clamp(round_div(total, 1000), -FIXED_SCALE, FIXED_SCALE)
            action_fixed.append(fixed)
            action_q1000.append(clamp(round_div(fixed * 1000, FIXED_SCALE), -1000, 1000))
        return action_fixed, action_q1000

    if state_q is None:
        state_q = [0] * hidden_size
    encoder_w_q, decoder_w_q, mingru_w_q = aligned_weight_slices_q(model)

    encoder_out = []
    for h in range(hidden_size):
        total = 0
        for i in range(OBS_SIZE):
            total += obs_q1000[i] * encoder_w_q[h * OBS_SIZE + i]
        encoder_out.append(clamp(round_div(total, 1000), -internal_limit, internal_limit))

    hidden = encoder_out
    if num_layers > 0:
        combined = []
        for o in range(3 * hidden_size):
            total = 0
            for h in range(hidden_size):
                total += encoder_out[h] * mingru_w_q[0][o * hidden_size + h]
            combined.append(clamp(round_div(total, FIXED_SCALE), -internal_limit, internal_limit))

        mingru_out = []
        for h in range(hidden_size):
            hidden_v = combined[h]
            gate = combined[hidden_size + h]
            highway = combined[2 * hidden_size + h]
            gate_s = sigmoid_q(gate)
            if hidden_v >= 0:
                hidden_tilde = hidden_v + FIXED_SCALE // 2
            else:
                hidden_tilde = sigmoid_q(hidden_v)
            next_state = state_q[h] + round_div(gate_s * (hidden_tilde - state_q[h]), FIXED_SCALE)
            next_state = clamp(next_state, -internal_limit, internal_limit)
            highway_s = sigmoid_q(highway)
            out = round_div(
                highway_s * next_state + (FIXED_SCALE - highway_s) * encoder_out[h],
                FIXED_SCALE,
            )
            out = clamp(out, -internal_limit, internal_limit)
            state_q[h] = next_state
            mingru_out.append(out)
        hidden = mingru_out

    for action in range(NUM_ACTIONS):
        total = 0
        for h in range(hidden_size):
            total += hidden[h] * decoder_w_q[action * hidden_size + h]
        fixed = clamp(round_div(total, FIXED_SCALE), -FIXED_SCALE, FIXED_SCALE)
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
    max_full_fixed_action_q1000_diff = 0
    max_fixed_fold_qobs_action_q1000_diff = 0
    full_fold_tick_mismatches = 0
    fixed_fold_qobs_tick_mismatches = 0
    fixed_full_tick_mismatches = 0
    fixed_full_max_tick_delta = 0
    examples = []

    for sample_name, obs, obs_q1000 in samples:
        count += 1
        obs_from_q = [value / 1000.0 for value in obs_q1000]
        full = forward_full(model, obs_from_q)
        full_exact_obs = None
        folded = None
        folded_qobs = None
        if model["folded"] is not None:
            full_exact_obs = forward_full(model, obs)
            folded = forward_folded(model, obs)
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
        folded_ticks = None
        folded_qobs_ticks = None
        full_exact_ticks = None
        if folded is not None:
            full_exact_ticks = [
                action_to_ticks(
                    value,
                    args.max_wheel_speed_mps,
                    args.command_deadband,
                    args.min_drive_ticks_per_sec,
                )
                for value in full_exact_obs
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
            full_q1000 = clamp(float_to_q1000(full[a]), -1000, 1000)
            max_full_fixed_action_q1000_diff = max(
                max_full_fixed_action_q1000_diff,
                abs(fixed_q1000[a] - full_q1000),
            )
            if folded is not None:
                max_full_fold_action_diff = max(
                    max_full_fold_action_diff,
                    abs(full_exact_obs[a] - folded[a]),
                )
                folded_q1000 = clamp(float_to_q1000(folded_qobs[a]), -1000, 1000)
                max_fixed_fold_qobs_action_q1000_diff = max(
                    max_fixed_fold_qobs_action_q1000_diff,
                    abs(fixed_q1000[a] - folded_q1000),
                )

        if folded_ticks is not None and full_exact_ticks != folded_ticks:
            full_fold_tick_mismatches += 1
        if folded_qobs_ticks is not None and fixed_ticks != folded_qobs_ticks:
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
            fixed_full_max_tick_delta = max(
                fixed_full_max_tick_delta,
                max(abs(fixed_ticks[a] - full_ticks[a]) for a in range(NUM_ACTIONS)),
            )
            if len(examples) < 5 and folded_qobs_ticks is None:
                examples.append(
                    (
                        sample_name,
                        obs_q1000,
                        [round(v, 6) for v in full],
                        fixed,
                        full_ticks,
                        fixed_ticks,
                    )
                )

    fixed_mismatch_frac = (
        fixed_fold_qobs_tick_mismatches / count if count > 0 else 0.0
    )
    if model["folded"] is not None:
        ok = (
            max_full_fold_action_diff <= args.max_fold_action_diff
            and full_fold_tick_mismatches == 0
            and fixed_mismatch_frac <= args.max_fixed_tick_mismatch_frac
        )
    else:
        ok = fixed_full_tick_mismatches / count <= args.max_fixed_tick_mismatch_frac

    print(f"\n{name}:")
    print(f"  samples={count}")
    if model["folded"] is not None:
        print(f"  full_vs_folded max_action_diff={max_full_fold_action_diff:.9g}")
        print(f"  full_vs_folded tick_mismatches={full_fold_tick_mismatches}")
        print(
            "  fixed_vs_folded_qobs "
            f"max_action_q1000_diff={max_fixed_fold_qobs_action_q1000_diff} "
            f"tick_mismatches={fixed_fold_qobs_tick_mismatches} "
            f"({100.0 * fixed_mismatch_frac:.3f}%)"
        )
    print(
        "  fixed_vs_full "
        f"max_action_q1000_diff={max_full_fixed_action_q1000_diff} "
        f"tick_mismatches={fixed_full_tick_mismatches} "
        f"max_tick_delta={fixed_full_max_tick_delta}"
    )
    for example in examples:
        sample_name, obs_q1000, reference, fixed, reference_ticks, fixed_ticks = example
        print(
            "  mismatch "
            f"{sample_name} obs_q1000={obs_q1000} "
            f"reference={reference} fixed={fixed} "
            f"reference_ticks={reference_ticks} fixed_ticks={fixed_ticks}"
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
    parser.add_argument("--max-fixed-tick-mismatch-frac", type=float, default=0.03)
    args = parser.parse_args()

    checkpoint = args.checkpoint or latest_checkpoint(root)
    if checkpoint is None:
        raise SystemExit("No line_follow checkpoint found")
    checkpoint = checkpoint.resolve()
    model = load_model(checkpoint, args.hidden_size, args.num_layers)

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
