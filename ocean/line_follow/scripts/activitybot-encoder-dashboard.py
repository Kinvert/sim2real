#!/usr/bin/env python3
"""Live ActivityBot encoder dashboard over propeller-load terminal mode."""

from __future__ import annotations

import argparse
import collections
import curses
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path


STREAM_RE = re.compile(r"^E\s+(-?\d+)\s+(-?\d+)\s+([01])\s+([01])")


class Sample:
    def __init__(self, t: float, left: int, right: int, left_raw: int, right_raw: int):
        self.t = t
        self.left = left
        self.right = right
        self.left_raw = left_raw
        self.right_raw = right_raw


def line_follow_root() -> Path:
    return Path(__file__).resolve().parents[1]


def pufferlib_root() -> Path:
    return Path(__file__).resolve().parents[3]


def build_firmware(line_root: Path) -> list[str]:
    result = subprocess.run(
        [str(line_root / "scripts/parallax-build-encoder-stream.sh")],
        check=True,
        capture_output=True,
        text=True,
    )
    return [line for line in result.stdout.splitlines() if line.strip()]


def reader_thread(proc: subprocess.Popen[str], out: queue.Queue[tuple[str, object]]) -> None:
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


def bar(value: float, width: int, peak: float) -> str:
    if width <= 0:
        return ""
    if peak <= 0:
        filled = 0
    else:
        filled = max(0, min(width, int(round((value / peak) * width))))
    return "#" * filled + "-" * (width - filled)


def spark(values: collections.deque[float], width: int, peak: float) -> str:
    if width <= 0:
        return ""
    chars = " .:-=+*#%@"
    recent = list(values)[-width:]
    if len(recent) < width:
        recent = [0.0] * (width - len(recent)) + recent
    out = []
    for value in recent:
        idx = 0 if peak <= 0 else int(max(0, min(len(chars) - 1, round((value / peak) * (len(chars) - 1)))))
        out.append(chars[idx])
    return "".join(out)


def draw_line(stdscr: curses.window, y: int, x: int, text: str, attr: int = 0) -> None:
    height, width = stdscr.getmaxyx()
    if 0 <= y < height and x < width:
        stdscr.addnstr(y, x, text, max(0, width - x - 1), attr)


def dashboard(stdscr: curses.window, args: argparse.Namespace) -> int:
    line_root = line_follow_root()
    repo_root = pufferlib_root()
    loader = repo_root / "tools/parallax/simpleide/opt/parallax/bin/propeller-load"
    elf = line_root / "build/parallax-smoke/encoder_stream.elf"

    try:
        curses.curs_set(0)
    except curses.error:
        pass
    stdscr.nodelay(True)
    stdscr.timeout(50)

    if args.build:
        draw_line(stdscr, 0, 0, "Building encoder stream firmware...")
        stdscr.refresh()
        build_logs = build_firmware(line_root)
    else:
        build_logs = []

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
        cwd=str(line_root),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        bufsize=1,
    )

    events: queue.Queue[tuple[str, object]] = queue.Queue()
    thread = threading.Thread(target=reader_thread, args=(proc, events), daemon=True)
    thread.start()

    logs: collections.deque[str] = collections.deque(build_logs[-6:], maxlen=6)
    samples: collections.deque[Sample] = collections.deque(maxlen=240)
    left_speed: collections.deque[float] = collections.deque(maxlen=180)
    right_speed: collections.deque[float] = collections.deque(maxlen=180)
    last = Sample(time.monotonic(), 0, 0, 0, 0)
    latest = last
    peak_speed = 1.0
    start = time.monotonic()
    exit_code: int | None = None

    try:
        while True:
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
                    peak_speed = max(peak_speed * 0.995, ls, rs, 1.0)
                    left_speed.append(ls)
                    right_speed.append(rs)
                    samples.append(sample)
                    latest = sample
                elif kind == "log":
                    logs.append(str(payload))
                elif kind == "exit":
                    exit_code = payload if isinstance(payload, int) else proc.poll()

            key = stdscr.getch()
            if key in (ord("q"), ord("Q"), 27):
                break
            if args.seconds and time.monotonic() - start >= args.seconds:
                break
            if exit_code is not None:
                logs.append(f"propeller-load exited: {exit_code}")
                break

            height, width = stdscr.getmaxyx()
            graph_w = max(10, width - 24)
            left_latest_speed = left_speed[-1] if left_speed else 0.0
            right_latest_speed = right_speed[-1] if right_speed else 0.0
            total = max(1, latest.left, latest.right)

            stdscr.erase()
            title = "ActivityBot Live Encoder Dashboard"
            draw_line(stdscr, 0, 0, title, curses.A_BOLD)
            draw_line(stdscr, 1, 0, f"Port {args.port} | RAM-only firmware | press q or Esc to quit")
            draw_line(stdscr, 3, 0, f"LEFT  count {latest.left:6d} raw {latest.left_raw} speed {left_latest_speed:6.1f} edges/s")
            draw_line(stdscr, 4, 0, f"      [{bar(latest.left, graph_w, total)}]")
            draw_line(stdscr, 6, 0, f"RIGHT count {latest.right:6d} raw {latest.right_raw} speed {right_latest_speed:6.1f} edges/s")
            draw_line(stdscr, 7, 0, f"      [{bar(latest.right, graph_w, total)}]")
            draw_line(stdscr, 9, 0, f"Speed trace, peak scale {peak_speed:0.1f} edges/s")
            draw_line(stdscr, 10, 0, "LEFT  " + spark(left_speed, graph_w, peak_speed))
            draw_line(stdscr, 11, 0, "RIGHT " + spark(right_speed, graph_w, peak_speed))
            draw_line(stdscr, 13, 0, "Loader / serial log:")
            for i, line in enumerate(logs):
                draw_line(stdscr, 14 + i, 0, line)
            if not samples:
                draw_line(stdscr, max(0, height - 2), 0, "Waiting for ENCODER_STREAM data from the robot...")
            else:
                draw_line(stdscr, max(0, height - 2), 0, "Spin either wheel by hand. The bars and traces update from P14/P15 encoder edges.")
            stdscr.refresh()

    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)

    if args.summary_file:
        summary = Path(args.summary_file)
        if not summary.is_absolute():
            summary = line_root / summary
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
                    f"samples={len(samples)}",
                ]
            )
            + "\n"
        )

    return 0 if exit_code in (None, 0) else int(exit_code)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=os.environ.get("PROPELLER_LOAD_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--board", default=os.environ.get("PROPELLER_LOAD_BOARD", "activityboard"))
    parser.add_argument("--seconds", type=float, default=0.0, help="Exit after N seconds; default runs until q/Esc.")
    parser.add_argument("--no-build", dest="build", action="store_false", help="Skip firmware build before loading.")
    parser.add_argument(
        "--summary-file",
        default="build/parallax-smoke/encoder-dashboard-last.txt",
        help="Write final dashboard counters here; relative paths are repo-relative.",
    )
    parser.set_defaults(build=True)
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    return curses.wrapper(dashboard, args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
