#!/usr/bin/env python3
"""Raylib live dashboard for ActivityBot encoder edges."""

from __future__ import annotations

import argparse
import collections
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pyray as rl


STREAM_RE = re.compile(r"^E\s+(-?\d+)\s+(-?\d+)\s+([01])\s+([01])")


class Sample:
    def __init__(self, t: float, left: int, right: int, left_raw: int, right_raw: int):
        self.t = t
        self.left = left
        self.right = right
        self.left_raw = left_raw
        self.right_raw = right_raw


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def build_firmware(root: Path) -> list[str]:
    result = subprocess.run(
        [str(root / "scripts/parallax-build-encoder-stream.sh")],
        check=True,
        capture_output=True,
        text=True,
    )
    return [line for line in result.stdout.splitlines() if line.strip()]


def read_loader(proc: subprocess.Popen[str], out: queue.Queue[tuple[str, object]]) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
        clean = line.replace("\r", "").strip()
        match = STREAM_RE.search(clean)
        if match:
            out.put((
                "sample",
                Sample(
                    time.monotonic(),
                    int(match.group(1)),
                    int(match.group(2)),
                    int(match.group(3)),
                    int(match.group(4)),
                ),
            ))
        elif clean:
            out.put(("log", clean))
    out.put(("exit", proc.poll()))


def draw_label(x: int, y: int, text: str, size: int = 22, color=None) -> None:
    rl.draw_text(text, x, y, size, color or rl.RAYWHITE)


def draw_bar(x: int, y: int, w: int, h: int, value: float, peak: float, color) -> None:
    rl.draw_rectangle(x, y, w, h, rl.Color(36, 39, 46, 255))
    fill = 0 if peak <= 0 else int(max(0, min(w, (value / peak) * w)))
    rl.draw_rectangle(x, y, fill, h, color)
    rl.draw_rectangle_lines(x, y, w, h, rl.Color(120, 126, 138, 255))


def draw_trace(x: int, y: int, w: int, h: int, values: collections.deque[float], peak: float, color) -> None:
    rl.draw_rectangle(x, y, w, h, rl.Color(22, 25, 31, 255))
    rl.draw_rectangle_lines(x, y, w, h, rl.Color(80, 86, 96, 255))
    rl.draw_line(x, y + h // 2, x + w, y + h // 2, rl.Color(45, 50, 58, 255))
    recent = list(values)[-w:]
    if len(recent) < 2:
        return
    start_x = x + max(0, w - len(recent))
    scale = max(peak, 1.0)
    prev_x = start_x
    prev_y = y + h - int((recent[0] / scale) * (h - 8)) - 4
    for idx, value in enumerate(recent[1:], start=1):
        px = start_x + idx
        py = y + h - int((value / scale) * (h - 8)) - 4
        py = max(y + 3, min(y + h - 3, py))
        rl.draw_line(prev_x, prev_y, px, py, color)
        prev_x, prev_y = px, py


def run(args: argparse.Namespace) -> int:
    root = repo_root()
    if args.build:
        build_logs = build_firmware(root)
    else:
        build_logs = []

    loader = root / "tools/parallax/simpleide/opt/parallax/bin/propeller-load"
    elf = root / "build/parallax-smoke/encoder_stream.elf"
    cmd = [
        str(loader),
        "-b",
        args.board,
        "-p",
        args.port,
        "-r",
        "-t",
        str(elf),
    ]

    proc = subprocess.Popen(
        cmd,
        cwd=str(root),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        bufsize=1,
    )
    events: queue.Queue[tuple[str, object]] = queue.Queue()
    threading.Thread(target=read_loader, args=(proc, events), daemon=True).start()

    rl.init_window(args.width, args.height, "ActivityBot Encoder Dashboard")
    rl.set_target_fps(60)

    logs: collections.deque[str] = collections.deque(build_logs[-4:], maxlen=7)
    left_speed: collections.deque[float] = collections.deque(maxlen=max(120, args.width - 160))
    right_speed: collections.deque[float] = collections.deque(maxlen=max(120, args.width - 160))
    latest = Sample(time.monotonic(), 0, 0, 0, 0)
    peak_speed = 1.0
    peak_count = 1
    start = time.monotonic()
    exit_code: int | None = None

    bg = rl.Color(15, 18, 24, 255)
    panel = rl.Color(27, 31, 39, 255)
    cyan = rl.Color(64, 196, 255, 255)
    orange = rl.Color(255, 176, 64, 255)
    green = rl.Color(94, 220, 126, 255)
    muted = rl.Color(155, 164, 178, 255)

    try:
        while not rl.window_should_close():
            while True:
                try:
                    kind, payload = events.get_nowait()
                except queue.Empty:
                    break
                if kind == "sample":
                    sample = payload
                    assert isinstance(sample, Sample)
                    dt = max(1e-3, sample.t - latest.t)
                    dl = max(0, sample.left - latest.left)
                    dr = max(0, sample.right - latest.right)
                    ls = dl / dt
                    rs = dr / dt
                    peak_speed = max(peak_speed * 0.997, ls, rs, 1.0)
                    peak_count = max(peak_count, sample.left, sample.right, 1)
                    left_speed.append(ls)
                    right_speed.append(rs)
                    latest = sample
                elif kind == "log":
                    logs.append(str(payload))
                elif kind == "exit":
                    exit_code = payload if isinstance(payload, int) else proc.poll()

            if rl.is_key_pressed(rl.KeyboardKey.KEY_Q) or rl.is_key_pressed(rl.KeyboardKey.KEY_ESCAPE):
                break
            if args.seconds and time.monotonic() - start >= args.seconds:
                break

            width = rl.get_screen_width()
            height = rl.get_screen_height()
            graph_w = width - 120

            rl.begin_drawing()
            rl.clear_background(bg)

            rl.draw_rectangle(28, 24, width - 56, 96, panel)
            draw_label(48, 42, "ActivityBot Encoder Dashboard", 32, rl.RAYWHITE)
            draw_label(50, 82, f"{args.port}  |  RAM-only  |  spin wheels by hand  |  Q/Esc closes", 20, muted)

            draw_label(48, 148, f"LEFT  edges {latest.left:6d}   raw P14={latest.left_raw}   speed {left_speed[-1] if left_speed else 0:6.1f}/s", 24, cyan)
            draw_bar(48, 184, graph_w, 34, latest.left, peak_count, cyan)

            draw_label(48, 250, f"RIGHT edges {latest.right:6d}   raw P15={latest.right_raw}   speed {right_speed[-1] if right_speed else 0:6.1f}/s", 24, orange)
            draw_bar(48, 286, graph_w, 34, latest.right, peak_count, orange)

            draw_label(48, 356, f"Scrolling speed trace   peak scale {peak_speed:0.1f} edges/s", 22, rl.RAYWHITE)
            draw_trace(48, 390, graph_w, 86, left_speed, peak_speed, cyan)
            draw_trace(48, 500, graph_w, 86, right_speed, peak_speed, orange)
            draw_label(60, 396, "LEFT", 18, cyan)
            draw_label(60, 506, "RIGHT", 18, orange)

            raw_y = 148
            rl.draw_circle(width - 54, raw_y + 12, 11, green if latest.left_raw else rl.Color(70, 75, 84, 255))
            rl.draw_circle(width - 54, raw_y + 114, 11, green if latest.right_raw else rl.Color(70, 75, 84, 255))

            log_y = height - 140
            draw_label(48, log_y, "loader / serial", 18, muted)
            for idx, line in enumerate(list(logs)[-5:]):
                draw_label(48, log_y + 24 + idx * 20, line[:110], 18, muted)

            if exit_code is not None:
                draw_label(48, height - 36, f"propeller-load exited: {exit_code}", 18, rl.RED)
            else:
                draw_label(48, height - 36, "live from P14/P15 through usbipd -> /dev/ttyUSB0 -> propeller-load terminal mode", 18, muted)

            rl.end_drawing()
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
        rl.close_window()

    if args.summary_file:
        summary = Path(args.summary_file)
        if not summary.is_absolute():
            summary = root / summary
        summary.parent.mkdir(parents=True, exist_ok=True)
        summary.write_text(
            "\n".join(
                [
                    f"port={args.port}",
                    f"board={args.board}",
                    f"left_edges={latest.left}",
                    f"right_edges={latest.right}",
                    f"left_raw={latest.left_raw}",
                    f"right_raw={latest.right_raw}",
                    f"peak_edges_per_second={peak_speed:.2f}",
                    "logs:",
                    *list(logs),
                ]
            )
            + "\n"
        )

    return 0 if exit_code in (None, 0) else int(exit_code)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=os.environ.get("PROPELLER_LOAD_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--board", default=os.environ.get("PROPELLER_LOAD_BOARD", "activityboard"))
    parser.add_argument("--width", type=int, default=1120)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--no-build", dest="build", action="store_false")
    parser.add_argument("--summary-file", default="build/parallax-smoke/encoder-dashboard-raylib-last.txt")
    parser.set_defaults(build=True)
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
