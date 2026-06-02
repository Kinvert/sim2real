#!/usr/bin/env python3
import importlib.util
import re
import struct
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def import_script(name, relpath):
    path = ROOT / relpath
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


eq = import_script(
    "line_follow_model_equivalence",
    "ocean/line_follow/scripts/line-follow-model-equivalence.py",
)


def q(values):
    return [round(value * eq.FIXED_SCALE) for value in values]


def parse_header_weights_q(path):
    text = path.read_text(encoding="ascii")
    match = re.search(
        r"line_follow_model_weights_q"
        r"\[LINE_FOLLOW_MODEL_PADDED_FLOATS\] = \{\n(.*?)\};",
        text,
        re.S,
    )
    if match is None:
        raise AssertionError("generated header did not contain weights_q array")
    return [int(value) for value in re.findall(r"-?\d+", match.group(1))]


def assert_slice(values, start, expected, label):
    actual = values[start : start + len(expected)]
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def main():
    build_dir = ROOT / "ocean/line_follow/build/tests"
    build_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = build_dir / "synthetic-h3l1-native.bin"
    compact_checkpoint = build_dir / "synthetic-h3l1-compact.bin"
    header = build_dir / "synthetic-h3l1-weights.h"

    encoder = [float(value) for value in range(1, 10)]
    decoder = [float(value) for value in range(101, 110)]
    log_std = [201.0, 202.0]
    mingru = [float(value) for value in range(301, 328)]
    compact = encoder + decoder + log_std + mingru
    compact_checkpoint.write_bytes(struct.pack("<" + "f" * len(compact), *compact))

    native = [0.0] * 67
    native[0:9] = encoder
    native[16:25] = decoder
    native[32:34] = log_std
    native[40:67] = mingru
    checkpoint.write_bytes(struct.pack("<" + "f" * len(native), *native))

    try:
        eq.load_model(compact_checkpoint, hidden_size=3, num_layers=1)
    except SystemExit:
        pass
    else:
        raise AssertionError("compact H3/L1 checkpoint should be rejected")

    model = eq.load_model(checkpoint, hidden_size=3, num_layers=1)
    assert model["encoder"] == encoder
    assert model["decoder"] == decoder
    assert model["mingru"] == [mingru]
    assert model["folded"] is None
    assert len(model["weights_q"]) == eq.aligned_read_float_count(3, 1)
    assert len(model["weights_q"]) == 72

    weights_q = model["weights_q"]
    assert_slice(weights_q, 0, q(encoder), "encoder")
    assert_slice(weights_q, 9, [0] * 7, "encoder padding")
    assert_slice(weights_q, 16, q(decoder), "decoder")
    assert_slice(weights_q, 25, [0] * 7, "decoder padding")
    assert_slice(weights_q, 32, q(log_std), "log_std")
    assert_slice(weights_q, 34, [0] * 6, "log_std padding")
    assert_slice(weights_q, 40, q(mingru), "mingru")
    assert_slice(weights_q, 67, [0] * 5, "mingru padding")

    subprocess.run(
        [
            sys.executable,
            str(ROOT / "ocean/line_follow/scripts/generate-line-follow-model-header.py"),
            "--hidden-size",
            "3",
            "--num-layers",
            "1",
            str(checkpoint),
            str(header),
        ],
        check=True,
    )
    header_text = header.read_text(encoding="ascii")
    assert "#define LINE_FOLLOW_MODEL_RAW_FLOATS 67" in header_text
    assert "#define LINE_FOLLOW_MODEL_PADDED_FLOATS 72" in header_text
    assert parse_header_weights_q(header) == weights_q


if __name__ == "__main__":
    main()
