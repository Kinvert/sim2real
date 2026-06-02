#!/usr/bin/env python3
"""Measure recurrent wheel bias for centered QTI observations.

This is a deployment-screening diagnostic for H4/L1 policies. It reuses the
fixed-point inference math from line-follow-model-equivalence.py so the output
matches the Propeller fixed recurrent path more closely than host float eval.
"""

import argparse
import importlib.util
from pathlib import Path


def load_equivalence_module():
    path = Path(__file__).with_name("line-follow-model-equivalence.py")
    spec = importlib.util.spec_from_file_location("line_follow_model_equivalence", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def latest_checkpoint(root):
    checkpoints = list((root / "checkpoints" / "line_follow").glob("*/*.bin"))
    if not checkpoints:
        raise SystemExit("No line_follow checkpoint found")
    return max(checkpoints, key=lambda path: path.stat().st_mtime)


def parse_values(text):
    if not text:
        return [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]
    values = []
    for item in text.split(","):
        item = item.strip()
        if item:
            values.append(max(0, min(1000, int(item))))
    if not values:
        raise SystemExit("--middle-values did not contain any integers")
    return values


def run_sequence(eq, model, obs_q1000, steps, args):
    state_q = [0] * args.hidden_size
    rows = []
    for step in range(steps):
        action_fixed, action_q1000 = eq.forward_fixed(model, obs_q1000, state_q)
        ticks = [
            eq.action_fixed_to_ticks(
                action_fixed[i],
                args.max_wheel_speed_mps,
                args.command_deadband,
                args.min_drive_ticks_per_sec,
            )
            for i in range(2)
        ]
        rows.append((step, action_q1000[0], action_q1000[1], ticks[0], ticks[1]))
    return rows


def summarize(rows):
    count = len(rows)
    mean_left = sum(row[3] for row in rows) / count
    mean_right = sum(row[4] for row in rows) / count
    mean_bias = mean_right - mean_left
    final = rows[-1]
    return mean_left, mean_right, mean_bias, final[3], final[4], final[4] - final[3]


def main():
    root = Path(__file__).resolve().parents[3]
    eq = load_equivalence_module()
    defaults = eq.read_env_defaults(root)

    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", nargs="?", type=Path)
    parser.add_argument("--hidden-size", type=int, default=4)
    parser.add_argument("--num-layers", type=int, default=1)
    parser.add_argument("--steps", type=int, default=16)
    parser.add_argument("--middle-values", default="")
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
    parser.add_argument("--warn-bias-ticks", type=float, default=12.0)
    args = parser.parse_args()

    checkpoint = args.checkpoint or latest_checkpoint(root)
    checkpoint = checkpoint.resolve()
    model = eq.load_model(checkpoint, args.hidden_size, args.num_layers)
    middle_values = parse_values(args.middle_values)

    print(f"checkpoint={checkpoint}")
    print(
        f"shape hidden={args.hidden_size} layers={args.num_layers} "
        f"steps={args.steps} max_speed={args.max_wheel_speed_mps} "
        f"deadband={args.command_deadband} min_ticks={args.min_drive_ticks_per_sec}"
    )
    print(
        "case,middle_q1000,mean_left_ticks,mean_right_ticks,mean_bias_right_minus_left,"
        "final_left_ticks,final_right_ticks,final_bias_right_minus_left"
    )

    worst_abs_bias = 0.0
    worst_label = ""
    for middle in middle_values:
        obs = [0, middle, 0]
        rows = run_sequence(eq, model, obs, args.steps, args)
        mean_left, mean_right, mean_bias, final_left, final_right, final_bias = summarize(rows)
        print(
            f"center,{middle},{mean_left:.3f},{mean_right:.3f},{mean_bias:.3f},"
            f"{final_left},{final_right},{final_bias}"
        )
        if abs(mean_bias) > worst_abs_bias:
            worst_abs_bias = abs(mean_bias)
            worst_label = f"center:{middle}"

    for name, obs in [
        ("all_white", [0, 0, 0]),
        ("balanced_edge_low", [100, 300, 100]),
        ("balanced_edge_mid", [200, 600, 200]),
        ("balanced_edge_high", [300, 900, 300]),
    ]:
        rows = run_sequence(eq, model, obs, args.steps, args)
        mean_left, mean_right, mean_bias, final_left, final_right, final_bias = summarize(rows)
        print(
            f"{name},{obs[1]},{mean_left:.3f},{mean_right:.3f},{mean_bias:.3f},"
            f"{final_left},{final_right},{final_bias}"
        )
        if abs(mean_bias) > worst_abs_bias:
            worst_abs_bias = abs(mean_bias)
            worst_label = name

    print(f"worst_abs_mean_bias_ticks={worst_abs_bias:.3f} case={worst_label}")
    if worst_abs_bias > args.warn_bias_ticks:
        print(
            f"WARNING center/balanced recurrent bias exceeds {args.warn_bias_ticks:.1f} ticks"
        )


if __name__ == "__main__":
    main()
