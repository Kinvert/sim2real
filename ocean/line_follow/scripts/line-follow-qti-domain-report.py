#!/usr/bin/env python3
import argparse
import configparser
import csv
import random
from collections import Counter
from pathlib import Path


SENSORS = ("left", "middle", "right")


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * pct / 100.0))
    return ordered[max(0, min(idx, len(ordered) - 1))]


def read_env_config(path):
    parser = configparser.ConfigParser()
    parser.read(path)
    env = parser["env"]

    def f(name, fallback):
        return float(env.get(name, fallback))

    return {
        "sensor_side_lateral_m": f("sensor_side_lateral_m", 0.018),
        "sensor_lateral_jitter_m": f("sensor_lateral_jitter_m", 0.004),
        "sensor_noise_std": f("sensor_noise_std", 10.0),
        "qti_threshold": f("qti_threshold", 0.5),
        "qti_white_time": f("qti_white_time", 40.0),
        "qti_black_time": f("qti_black_time", 350.0),
        "qti_white_jitter": f("qti_white_jitter", 25.0),
        "qti_black_jitter": f("qti_black_jitter", 120.0),
        "qti_timeout": f("qti_timeout", 2500.0),
        "line_width_m": f("line_width_m", 0.018),
        "line_width_jitter_m": f("line_width_jitter_m", 0.006),
        "line_edge_softness_m": f("line_edge_softness_m", 0.006),
        "line_edge_softness_jitter_m": f("line_edge_softness_jitter_m", 0.004),
        "line_reflectance_noise": f("line_reflectance_noise", 0.75),
        "start_lateral_offset_m": f("start_lateral_offset_m", 0.025),
    }


def line_coverage(signed_lateral_m, line_width_m, edge_softness_m):
    dist = abs(signed_lateral_m)
    half_width = 0.5 * line_width_m
    if edge_softness_m <= 1e-6:
        return 1.0 if dist <= half_width else 0.0

    inner = max(0.0, half_width - edge_softness_m)
    outer = half_width + edge_softness_m
    if dist <= inner:
        return 1.0
    if dist >= outer:
        return 0.0

    t = (dist - inner) / (outer - inner)
    smooth = t * t * (3.0 - 2.0 * t)
    return 1.0 - smooth


def rand_signed(rng):
    return 2.0 * rng.random() - 1.0


def sample_episode_params(cfg, rng):
    width = max(
        0.004,
        cfg["line_width_m"] + cfg["line_width_jitter_m"] * rand_signed(rng),
    )
    softness = max(
        0.001,
        cfg["line_edge_softness_m"]
        + cfg["line_edge_softness_jitter_m"] * rand_signed(rng),
    )
    softness = min(softness, 0.75 * width)
    return width, softness


def sample_sensor_layout(cfg, rng):
    base = [cfg["sensor_side_lateral_m"], 0.0, -cfg["sensor_side_lateral_m"]]
    return [
        lateral + cfg["sensor_lateral_jitter_m"] * rand_signed(rng)
        for lateral in base
    ]


def sample_sensor_response(cfg, rng, robot_lateral_m):
    width, softness = sample_episode_params(cfg, rng)
    sensor_laterals = sample_sensor_layout(cfg, rng)
    reflectance_noise = min(max(cfg["line_reflectance_noise"], 0.0), 0.90)
    black_reflectance = 1.0 - reflectance_noise * rng.random()
    raw = []
    obs = []
    coverage = []
    for lateral in sensor_laterals:
        white = max(
            0.0,
            cfg["qti_white_time"] + cfg["qti_white_jitter"] * rand_signed(rng),
        )
        black = max(
            white + 1.0,
            cfg["qti_black_time"] + cfg["qti_black_jitter"] * rand_signed(rng),
        )
        black = min(black, cfg["qti_timeout"])
        black = max(black, white + 1.0)
        cov = line_coverage(robot_lateral_m + lateral, width, softness)
        reading = white + cov * black_reflectance * (black - white)
        reading += cfg["sensor_noise_std"] * rand_signed(rng)
        reading = clamp(reading, 0.0, cfg["qti_timeout"])
        normalized = clamp(
            (reading - cfg["qti_white_time"])
            / (cfg["qti_black_time"] - cfg["qti_white_time"]),
            0.0,
            1.0,
        )
        raw.append(reading)
        obs.append(1000.0 * normalized)
        coverage.append(cov)

    flags = 0
    threshold = 1000.0 * cfg["qti_threshold"]
    for i, value in enumerate(obs):
        if value >= threshold:
            flags |= 1 << i

    return {
        "raw": raw,
        "obs": obs,
        "coverage": coverage,
        "flags": flags,
        "width_m": width,
        "softness_m": softness,
        "offset_m": robot_lateral_m,
    }


def start_sampler_offset(cfg, sensor_laterals, width, rng):
    targets = [
        -sensor_laterals[0],
        -0.5 * (sensor_laterals[0] + sensor_laterals[1]),
        -sensor_laterals[1],
        -0.5 * (sensor_laterals[1] + sensor_laterals[2]),
        -sensor_laterals[2],
    ]
    offset = rng.choice(targets)
    offset += 0.25 * width * rand_signed(rng)
    limit = max(cfg["start_lateral_offset_m"], 0.0)
    return clamp(offset, -limit, limit)


def sample_start_response(cfg, rng):
    width, softness = sample_episode_params(cfg, rng)
    sensor_laterals = sample_sensor_layout(cfg, rng)
    offset = start_sampler_offset(cfg, sensor_laterals, width, rng)
    reflectance_noise = min(max(cfg["line_reflectance_noise"], 0.0), 0.90)
    black_reflectance = 1.0 - reflectance_noise * rng.random()

    raw = []
    obs = []
    coverage = []
    for lateral in sensor_laterals:
        white = max(
            0.0,
            cfg["qti_white_time"] + cfg["qti_white_jitter"] * rand_signed(rng),
        )
        black = max(
            white + 1.0,
            cfg["qti_black_time"] + cfg["qti_black_jitter"] * rand_signed(rng),
        )
        black = min(black, cfg["qti_timeout"])
        black = max(black, white + 1.0)
        cov = line_coverage(offset + lateral, width, softness)
        reading = white + cov * black_reflectance * (black - white)
        reading += cfg["sensor_noise_std"] * rand_signed(rng)
        reading = clamp(reading, 0.0, cfg["qti_timeout"])
        normalized = clamp(
            (reading - cfg["qti_white_time"])
            / (cfg["qti_black_time"] - cfg["qti_white_time"]),
            0.0,
            1.0,
        )
        raw.append(reading)
        obs.append(1000.0 * normalized)
        coverage.append(cov)

    flags = 0
    threshold = 1000.0 * cfg["qti_threshold"]
    for i, value in enumerate(obs):
        if value >= threshold:
            flags |= 1 << i

    return {
        "raw": raw,
        "obs": obs,
        "coverage": coverage,
        "flags": flags,
        "width_m": width,
        "softness_m": softness,
        "offset_m": offset,
    }


def summarize_rows(name, rows, raw_key, obs_key, coverage_key=None):
    print(f"\n## {name}")
    print(f"rows: {len(rows)}")
    if not rows:
        return

    flags = Counter(int(row["flags"]) for row in rows)
    flag_text = ", ".join(
        f"{flag}:{count} ({100.0 * count / len(rows):.1f}%)"
        for flag, count in sorted(flags.items())
    )
    print(f"flags: {flag_text}")

    for i, sensor in enumerate(SENSORS):
        raw = [float(raw_key(row, i, sensor)) for row in rows]
        obs = [float(obs_key(row, i, sensor)) for row in rows]
        print(
            f"{sensor:>6} raw p05/p50/p95 "
            f"{percentile(raw, 5):6.1f} {percentile(raw, 50):6.1f} {percentile(raw, 95):6.1f} "
            f"obs_q1000 p05/p50/p95 "
            f"{percentile(obs, 5):6.1f} {percentile(obs, 50):6.1f} {percentile(obs, 95):6.1f}"
        )
        if coverage_key is not None:
            cov = [float(coverage_key(row, i, sensor)) for row in rows]
            print(
                f"{'':>6} coverage p05/p50/p95 "
                f"{percentile(cov, 5):6.2f} {percentile(cov, 50):6.2f} {percentile(cov, 95):6.2f}"
            )


def robot_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def sim_case_rows(cfg, rng, samples, case):
    side = cfg["sensor_side_lateral_m"]
    if case == "start_sampler":
        return [sample_start_response(cfg, rng) for _ in range(samples)]
    if case == "uniform_visible":
        span = side + 0.018
        return [
            sample_sensor_response(cfg, rng, (2.0 * rng.random() - 1.0) * span)
            for _ in range(samples)
        ]
    if case == "center_middle":
        return [
            sample_sensor_response(cfg, rng, 0.002 * rand_signed(rng))
            for _ in range(samples)
        ]
    if case == "between_left_middle":
        return [
            sample_sensor_response(cfg, rng, -0.5 * side + 0.002 * rand_signed(rng))
            for _ in range(samples)
        ]
    if case == "between_middle_right":
        return [
            sample_sensor_response(cfg, rng, 0.5 * side + 0.002 * rand_signed(rng))
            for _ in range(samples)
        ]
    raise ValueError(f"unknown case {case}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/line_follow.ini")
    parser.add_argument(
        "--robot-csv",
        default="ocean/line_follow/build/line-follow/lf000_20260529.csv",
    )
    parser.add_argument("--samples", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    cfg = read_env_config(args.config)
    rng = random.Random(args.seed)

    print("# Line Follow QTI Domain Report")
    print(f"config: {Path(args.config).resolve()}")
    print(f"samples_per_case: {args.samples}")
    print(
        "nominal: "
        f"white={cfg['qti_white_time']:.1f} black={cfg['qti_black_time']:.1f} "
        f"line_width={1000.0 * cfg['line_width_m']:.1f}mm "
        f"edge_softness={1000.0 * cfg['line_edge_softness_m']:.1f}mm "
        f"reflectance_noise={cfg['line_reflectance_noise']:.2f} "
        f"sensor_side={1000.0 * cfg['sensor_side_lateral_m']:.1f}mm"
    )

    robot_path = Path(args.robot_csv)
    if robot_path.exists():
        summarize_rows(
            "Robot SD log",
            robot_rows(robot_path),
            lambda row, _i, sensor: row[f"raw_{sensor}"],
            lambda row, _i, sensor: row[f"obs_{sensor}_q1000"],
        )
    else:
        print(f"\nRobot CSV not found: {robot_path}")

    cases = [
        ("Sim reset start sampler", "start_sampler"),
        ("Sim visible lateral sweep", "uniform_visible"),
        ("Sim middle sensor centered", "center_middle"),
        ("Sim between left/middle half-edge", "between_left_middle"),
        ("Sim between middle/right half-edge", "between_middle_right"),
    ]
    for title, case in cases:
        rows = sim_case_rows(cfg, rng, args.samples, case)
        summarize_rows(
            title,
            rows,
            lambda row, i, _sensor: row["raw"][i],
            lambda row, i, _sensor: row["obs"][i],
            lambda row, i, _sensor: row["coverage"][i],
        )


if __name__ == "__main__":
    main()
