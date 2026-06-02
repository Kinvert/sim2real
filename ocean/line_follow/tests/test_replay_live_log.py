#!/usr/bin/env python3
import struct
import subprocess
import sys
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[3]
    out_dir = root / "ocean" / "line_follow" / "build" / "tests"
    out_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = out_dir / "zero-h8l0.bin"
    checkpoint.write_bytes(struct.pack("<50f", *([0.0] * 50)))
    log_path = out_dir / "synthetic-live-edge.txt"
    log_path.write_text(
        "\n".join(
            [
                "LINE_FOLLOW_MODEL_LIVE v1 drive=0",
                f"checkpoint={checkpoint}",
                "L step raw0 raw1 raw2 min0 min1 min2 max0 max1 max2 sensor0 sensor1 sensor2 model0 model1 model2 action0 action1 left right dt_ms period_us body_us qti_us policy_us drive_us qti_sample_us",
                "L 0 62 62 83 62 62 83 62 62 83 0 0 11 0 0 11  0.000  0.000 6 6 1 22000 800 300 400 6 1085",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    replay = root / "ocean" / "line_follow" / "scripts" / "line-follow-replay-live-log.py"
    result = subprocess.run(
        [
            sys.executable,
            str(replay),
            str(log_path),
            "--checkpoint",
            str(checkpoint),
            "--min-drive-ticks-per-sec",
            "6",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    stdout = result.stdout
    required = [
        "absolute: rows=1 ticks_left=6..6 ticks_right=6..6",
        "both_floor=1",
        "rawdiff_gain3: rows=1 ticks_left=6..6 ticks_right=6..6",
        "right=234..234",
    ]
    for text in required:
        if text not in stdout:
            raise AssertionError(f"missing replay output: {text}\n{stdout}")
    print("line-follow-replay-live-log test passed")


if __name__ == "__main__":
    main()
