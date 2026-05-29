#!/usr/bin/env python3
import argparse
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path


def repo_paths():
    script = Path(__file__).resolve()
    line_follow_root = script.parents[1]
    repo_root = line_follow_root.parents[1]
    return repo_root, line_follow_root


def ensure_port(port):
    path = Path(port)
    name = path.name
    sysfs = Path("/sys/class/tty") / name / "dev"

    if path.exists():
        subprocess.run(["sudo", "-n", "chmod", "666", port], check=False)
        return

    if not sysfs.exists():
        raise SystemExit(
            f"{port} is not present. Detach/attach the ActivityBot USB device and retry."
        )

    major_minor = sysfs.read_text(encoding="ascii").strip()
    major, minor = major_minor.split(":", 1)
    subprocess.run(
        ["sudo", "-n", "mknod", port, "c", major, minor],
        check=True,
    )
    subprocess.run(["sudo", "-n", "chmod", "666", port], check=True)


def run_build(line_follow_root, env):
    build_script = line_follow_root / "scripts" / "parallax-build-line-follow-sd-dump.sh"
    subprocess.run([str(build_script)], cwd=line_follow_root.parents[1], env=env, check=True)


def terminate(proc):
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2)


def extract_file(args, repo_root, line_follow_root, base_env, sd_file, output, serial_log):
    parallax_root = repo_root / "tools" / "parallax" / "simpleide" / "opt" / "parallax"
    propeller_load = parallax_root / "bin" / "propeller-load"
    elf = line_follow_root / "build" / "line-follow" / "line_follow_sd_dump.elf"
    env = base_env.copy()
    env["LINE_FOLLOW_SD_FILE"] = sd_file
    board = env.get("PROPELLER_LOAD_BOARD", "activityboard")

    ensure_port(args.port)
    run_build(line_follow_root, env)

    cmd = [
        str(propeller_load),
        "-b",
        board,
        "-p",
        args.port,
        "-r",
        "-t",
        str(elf),
    ]

    print(f"Loading {elf} to RAM on {args.port}")
    print("This does not write EEPROM and does not command motors.")
    print(f"Decoding {sd_file} to {output}")

    data = bytearray()
    raw_lines = []
    expected_bytes = None
    saw_begin = False
    deadline = time.monotonic() + args.timeout
    proc = subprocess.Popen(
        cmd,
        cwd=repo_root,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
    )

    assert proc.stdout is not None
    try:
        while True:
            if time.monotonic() > deadline:
                raise SystemExit(f"Timed out after {args.timeout}s before dump completed")

            line = proc.stdout.readline()
            if line == "":
                if proc.poll() is not None:
                    break
                time.sleep(0.02)
                continue

            raw_lines.append(line)
            stripped = line.strip()
            if args.print_hex or not stripped.startswith("H "):
                sys.stdout.write(line)
                sys.stdout.flush()

            if stripped.startswith("LF_SD_DUMP_BEGIN"):
                saw_begin = True
                continue
            if stripped.startswith("LF_SD_DUMP_ERROR"):
                raise SystemExit(stripped)
            if stripped.startswith("H "):
                hex_text = stripped[2:].strip()
                try:
                    data.extend(bytes.fromhex(hex_text))
                except ValueError as exc:
                    raise SystemExit(f"Bad hex line from robot: {stripped}") from exc
                continue
            if stripped.startswith("LF_SD_DUMP_END"):
                match = re.search(r"bytes=(\d+)", stripped)
                if match:
                    expected_bytes = int(match.group(1))
                break
    finally:
        terminate(proc)

    if not saw_begin:
        raise SystemExit("Robot did not report LF_SD_DUMP_BEGIN")
    if expected_bytes is None:
        raise SystemExit("Robot did not report LF_SD_DUMP_END bytes=N")
    if expected_bytes != len(data):
        raise SystemExit(
            f"Decoded {len(data)} bytes but robot reported {expected_bytes} bytes"
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(data)
    if serial_log:
        serial_log.parent.mkdir(parents=True, exist_ok=True)
        serial_log.write_text("".join(raw_lines), encoding="utf-8")

    print(f"Wrote {len(data)} bytes to {output}")
    if serial_log:
        print(f"Serial transcript: {serial_log}")
    return bytes(data)


def analyze(args, repo_root, line_follow_root):
    if args.no_analyze:
        return

    analyzer = line_follow_root / "scripts" / "analyze-line-follow-sd-log.py"
    cmd = [sys.executable, str(analyzer), str(args.output)]
    if args.csv:
        cmd.extend(["--csv", str(args.csv)])
    subprocess.run(cmd, cwd=repo_root, check=True)


def main():
    repo_root, line_follow_root = repo_paths()
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    default_out = line_follow_root / "build" / "line-follow" / f"lf_log_{timestamp}.bin"
    default_serial = (
        line_follow_root / "build" / "line-follow" / f"lf_sd_dump_{timestamp}.txt"
    )
    default_last_out = (
        line_follow_root / "build" / "line-follow" / f"lf_last_{timestamp}.txt"
    )
    default_last_serial = (
        line_follow_root / "build" / "line-follow" / f"lf_sd_dump_last_{timestamp}.txt"
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=os.environ.get("PROPELLER_LOAD_PORT", "/dev/ttyUSB0"))
    parser.add_argument(
        "--sd-file",
        default=os.environ.get("LINE_FOLLOW_SD_FILE", "auto"),
        help="File to extract from SD, or 'auto' to read lf_last.txt first",
    )
    parser.add_argument("--output", type=Path, default=default_out)
    parser.add_argument("--serial-log", type=Path, default=default_serial)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--no-analyze", action="store_true")
    parser.add_argument("--print-hex", action="store_true")
    args = parser.parse_args()

    env = os.environ.copy()
    parallax_root = repo_root / "tools" / "parallax" / "simpleide" / "opt" / "parallax"
    env["PATH"] = str(parallax_root / "bin") + os.pathsep + env.get("PATH", "")
    env.setdefault("PROPELLER_LOAD_BOARD", "activityboard")

    if args.sd_file == "auto":
        last_bytes = extract_file(
            args,
            repo_root,
            line_follow_root,
            env,
            "lf_last.txt",
            default_last_out,
            default_last_serial,
        )
        selected = last_bytes.decode("ascii", "replace").strip().splitlines()
        if not selected:
            raise SystemExit("lf_last.txt was empty")
        args.sd_file = selected[0].strip()
        if not re.match(r"^[A-Za-z0-9_.-]{1,12}$", args.sd_file):
            raise SystemExit(f"Unsafe SD filename in lf_last.txt: {args.sd_file!r}")
        print(f"Latest SD log according to lf_last.txt: {args.sd_file}")

    extract_file(
        args,
        repo_root,
        line_follow_root,
        env,
        args.sd_file,
        args.output,
        args.serial_log,
    )
    analyze(args, repo_root, line_follow_root)


if __name__ == "__main__":
    main()
