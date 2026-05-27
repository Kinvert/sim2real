# ActivityBot USB/IP Workflow Notes

This note covers the annoying part of the current g240 workflow: the ActivityBot
USB serial adapter is physically plugged into the Windows 11 host, while training,
firmware builds, and Codex live in g240 WSL2.

## Current Working Path

```text
5090 workstation
  -> SSH into g240 WSL2
  -> Propeller GCC / propeller-load in /home/claude/sim2real
  -> /dev/ttyUSB0 in WSL
  -> usbipd-win USB/IP bridge
  -> g240 Windows USB host
  -> ActivityBot FTDI USB Serial Converter
```

More explicitly, there are several directions of traffic:

- SSH command/control from the 5090 into WSL.
- Propeller program load and serial commands from WSL down to the robot.
- Robot serial telemetry back up from the Propeller to WSL.
- Raylib rendering from a WSL Python process out to g240 display `:0`.
- Windows-side `usbipd` control for attach/detach.

```text
+--------------------+        SSH commands         +--------------------------+
| 5090 Ubuntu desk   | --------------------------> | g240 WSL2 Ubuntu shell   |
| user + Codex client| <-------------------------- | command output/logs      |
+--------------------+                            +------------+-------------+
                                                                 |
                                                                 | starts WSL processes
                                                                 v
                                      +--------------------------+------------------+
                                      | /home/claude/sim2real in WSL               |
                                      |                                            |
                                      | propeller-load: program load + serial I/O  |
                                      | raylib Python: parses serial + draws UI    |
                                      +-------+--------------------------+---------+
                                              |                          |
                                              | draw calls / X11 / GLFW   | serial bytes
                                              v                          v
                                      +-------+--------+        +--------+---------+
                                      | g240 display   |        | /dev/ttyUSB0     |
                                      | DISPLAY=:0     |        | Linux ftdi_sio   |
                                      +----------------+        +--------+---------+
                                                                         ^
                                                                         |
                                                 robot telemetry         | program load/reset
                                                 comes back up           | goes down
                                                                         v
                                                               +---------+--------+
                                                               | WSL USB/IP       |
                                                               | vhci_hcd         |
                                                               +---------+--------+
                                                                         ^
                                                                         |
                                                                         v
                                                               +---------+--------+
                                                               | g240 Windows 11  |
                                                               | usbipd-win       |
                                                               +---------+--------+
                                                                         ^
                                                                         |
                                                   usbipd attach/detach   | physical USB packets
                                                   controlled here        v
                                                               +---------+--------+
                                                               | FTDI USB Serial  |
                                                               | ActivityBot P1   |
                                                               +------------------+
```

The important subtlety: the raylib dashboard is not a Windows program. It is
`/home/claude/sim2real/.venv/bin/python` running inside WSL. It displays on the
g240 monitor because `DISPLAY=:0` points at the WSL graphics/X11 path for the
Windows desktop. The robot data path still comes through `/dev/ttyUSB0` in WSL.

What runs where:

| Thing | Runs On | Role |
| --- | --- | --- |
| User shell / Codex client | 5090 Ubuntu | Sends SSH commands to g240 |
| `sshd` session | g240 WSL2 Ubuntu | Receives commands from 5090 |
| `propeller-elf-gcc` | g240 WSL2 Ubuntu | Builds Propeller C firmware |
| `propeller-load` | g240 WSL2 Ubuntu | Loads RAM firmware and bridges serial terminal output |
| `/dev/ttyUSB0` | g240 WSL2 Ubuntu | Linux serial endpoint for ActivityBot FTDI |
| `ftdi_sio` | g240 WSL2 Linux kernel | FTDI USB serial driver |
| `vhci_hcd` | g240 WSL2 Linux kernel | USB/IP virtual host controller |
| `usbipd-win` | g240 Windows 11 | Owns attach/detach of the physical USB device into WSL |
| `activitybot-encoder-dashboard-raylib.py` | g240 WSL2 Ubuntu | Reads `propeller-load` output and renders the dashboard |
| raylib/GLFW/X11 display | g240 WSL2 graphics path to Windows desktop | Shows the dashboard on g240 display `:0` |
| Propeller P1 firmware | ActivityBot | Emits serial encoder data after RAM load |

Verified hardware identity:

```text
Windows device: USB Serial Converter
VID:PID:        0403:6015
usbipd BUSID:   1-6
WSL device:     /dev/ttyUSB0
```

The official Microsoft WSL USB docs say WSL2 does not provide native USB
attachment by itself; the supported path is `usbipd-win`, which is controlled
from the Windows side with `usbipd bind`, `usbipd attach --wsl`, and
`usbipd detach` ([Microsoft WSL USB docs](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)).
Those docs specifically call out flashing Arduino-style boards as a target use
case for USB/IP.

The current verified firmware smoke test is:

```bash
timeout 12s sudo -n scripts/parallax-run-smoke.sh /dev/ttyUSB0
```

Expected successful output includes:

```text
Propeller Version 1 on /dev/ttyUSB0
Loading ... to hub memory
Verifying RAM ... OK
sim2real Parallax smoke test
```

When this works, the ActivityBot / FTDI RX/TX LEDs blink.

The current verified raylib display demo is:

```bash
DISPLAY=:0 sg dialout -c 'cd /home/claude/sim2real && .venv/bin/python scripts/activitybot-encoder-dashboard-raylib.py --port /dev/ttyUSB0 --seconds 180'
```

The successful run loaded RAM-only firmware, verified RAM OK, opened a raylib
window on the g240 display, and wrote:

```text
left_edges=196
right_edges=300
peak_edges_per_second=9.71
```

to `build/parallax-smoke/encoder-dashboard-raylib-last.txt`.

## Commands That Worked

Check the repo branch:

```bash
git status --short --branch
```

Verified branch output:

```text
## 4.0...origin/4.0
```

Check the Propeller compiler:

```bash
tools/parallax/simpleide/opt/parallax/bin/propeller-elf-gcc --version
```

Verified output:

```text
propeller-elf-gcc (propellergcc_v1_0_0_2411) 4.6.1
```

Check loader help and supported flags:

```bash
tools/parallax/simpleide/opt/parallax/bin/propeller-load -h
```

This verified flags such as `-p`, `-P`, `-Q`, `-e`, `-r`, `-t`, and `-f`.

Build the basic smoke firmware:

```bash
scripts/parallax-build-smoke.sh
```

Observed successful output:

```text
text    data     bss     dec     hex filename
7924     240     208    8372    20b4 /home/claude/sim2real/build/parallax-smoke/hello.elf
Built /home/claude/sim2real/build/parallax-smoke/hello.elf
```

Check WSL serial visibility:

```bash
scripts/parallax-check.sh
```

Observed successful device state after attach:

```text
Linux serial devices visible to WSL:
crw-rw---- 1 root dialout 188, 0 ... /dev/ttyUSB0

Propeller loader port scan:
/dev/ttyUSB0
```

Windows-side first attach that worked:

```powershell
usbipd bind --busid 1-6
usbipd attach --wsl --busid 1-6
usbipd list
```

Windows-side recovery that worked after the USB/IP path wedged:

```powershell
usbipd detach --busid 1-6
usbipd attach --wsl --busid 1-6
```

Known-good RAM-only smoke loader with sudo:

```bash
timeout 12s sudo -n scripts/parallax-run-smoke.sh /dev/ttyUSB0
```

Observed successful output:

```text
Propeller Version 1 on /dev/ttyUSB0
Loading /home/claude/sim2real/build/parallax-smoke/hello.elf to hub memory
8164 bytes sent
Verifying RAM ... OK
[ Entering terminal mode. Type ESC or Control-C to exit. ]
sim2real Parallax smoke test
```

Known-good RAM-only smoke loader through the `dialout` group:

```bash
timeout 12s sg dialout -c 'scripts/parallax-run-smoke.sh /dev/ttyUSB0'
```

This also loaded RAM, verified RAM OK, and printed the smoke-test message.

Add the WSL user to `dialout`:

```bash
sudo -n usermod -aG dialout claude
```

Use `dialout` from the current shell before logging in again:

```bash
sg dialout -c 'id && ls -l /dev/ttyUSB0'
```

Build the direct encoder monitor:

```bash
scripts/parallax-build-encoder-monitor.sh
```

Observed successful output:

```text
text    data     bss     dec     hex filename
8216     360     236    8812    226c /home/claude/sim2real/build/parallax-smoke/encoder_monitor.elf
Built /home/claude/sim2real/build/parallax-smoke/encoder_monitor.elf
```

Run the direct encoder monitor:

```bash
timeout 45s sg dialout -c 'scripts/parallax-run-encoder-monitor.sh /dev/ttyUSB0'
```

Observed successful output shape:

```text
ActivityBot encoder monitor
Spin wheels by hand. P14=left encoder, P15=right encoder.
left_raw right_raw left_edges right_edges
...
0 1 13 46
```

Build the machine-readable encoder stream:

```bash
scripts/parallax-build-encoder-stream.sh
```

Observed successful output:

```text
text    data     bss     dec     hex filename
8236     244     236    8716    220c /home/claude/sim2real/build/parallax-smoke/encoder_stream.elf
Built /home/claude/sim2real/build/parallax-smoke/encoder_stream.elf
```

Install raylib into the local venv:

```bash
UV_CACHE_DIR=/home/claude/sim2real/.uv-cache uv pip install --python /home/claude/sim2real/.venv/bin/python raylib
```

Observed installed packages:

```text
cffi==2.0.0
pycparser==3.0
raylib==5.5.0.4
```

Verify raylib import:

```bash
.venv/bin/python - <<'PY'
import pyray as rl
print('pyray-ok', rl)
PY
```

Run the raylib encoder dashboard on g240 display 0:

```bash
DISPLAY=:0 sg dialout -c 'cd /home/claude/sim2real && .venv/bin/python scripts/activitybot-encoder-dashboard-raylib.py --port /dev/ttyUSB0 --seconds 180'
```

Observed successful raylib/display output:

```text
INFO: DISPLAY: Device initialized successfully
INFO:     > Display size: 1920 x 1080
INFO:     > Screen size:  1120 x 720
INFO: PLATFORM: DESKTOP (GLFW - X11): Initialized successfully
```

Observed successful dashboard summary:

```text
port=/dev/ttyUSB0
board=activityboard
left_edges=196
right_edges=300
left_raw=0
right_raw=0
peak_edges_per_second=9.71
logs:
8480 bytes sent
Verifying RAM ... OK
[ Entering terminal mode. Type ESC or Control-C to exit. ]
ENCODER_STREAM v1
```

## Commands That Failed Or Were Not Enough

Running Windows programs from this SSH-launched WSL session failed:

```bash
/mnt/c/Windows/System32/cmd.exe /C ver
"/mnt/c/Program Files/usbipd-win/usbipd.exe" --version
```

Observed error:

```text
UtilConnectUnix:533: connect failed 1
UtilBindVsockAnyPort:307: socket failed 1
```

Trying visible WSL interop sockets also failed:

```bash
for s in /run/WSL/*_interop; do
  WSL_INTEROP="$s" /mnt/c/Windows/System32/cmd.exe /C ver
done
```

Plain `/dev/ttyUSB0` existence was not enough. The failure state looked like:

```text
scripts/parallax-check.sh sees /dev/ttyUSB0
propeller-load hangs before "Propeller Version 1"
RX/TX LEDs do not blink
```

That is when the Windows-side detach/attach recovery was needed.

## Failure Mode We Saw

The bad state is:

```text
/dev/ttyUSB0 exists in WSL
usbipd list on Windows may still say attached
propeller-load hangs before "Propeller Version 1"
ActivityBot / FTDI RX/TX LEDs do not blink
```

This is not a raylib problem. It means the Windows USB/IP to WSL FTDI serial
path is wedged before useful Propeller loader traffic starts.

WSL `dmesg` showed errors like:

```text
ftdi_sio ttyUSB0: ftdi_set_termios FAILED to set databits/stopbits/parity
ftdi_sio ttyUSB0: ftdi_set_termios urb failed to set baudrate
ftdi_sio ttyUSB0: failed to set flow control: -62
vhci_hcd: connection closed
usb 1-1: USB disconnect
```

The reliable recovery we verified was a Windows-side USB/IP detach and attach:

```powershell
usbipd detach --busid 1-6
usbipd attach --wsl --busid 1-6
```

After that, the known-good sudo smoke test worked again and RX/TX blinked.

## Why This Cannot Be Purely Solved In WSL

The attach/detach control plane belongs to `usbipd-win`, which runs on Windows.
Microsoft's WSL USB docs show `usbipd bind`, `usbipd attach --wsl`, and
`usbipd detach` as Windows PowerShell commands, then Linux sees the result via
normal tools such as `lsusb` and `/dev/ttyUSB*`
([Microsoft WSL USB docs](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)).

In principle, WSL can launch Windows executables. Microsoft documents that WSL
can run Windows tools by calling `[tool-name].exe`, and Windows can run Linux
commands through `wsl.exe` ([Microsoft WSL interop docs](https://learn.microsoft.com/en-us/windows/wsl/filesystems)).

In this current SSH-launched WSL session, direct Windows interop is broken:

```bash
/mnt/c/Windows/System32/cmd.exe /C ver
"/mnt/c/Program Files/usbipd-win/usbipd.exe" --version
```

both fail with WSL interop socket errors:

```text
UtilConnectUnix:533: connect failed 1
UtilBindVsockAnyPort:307: socket failed 1
```

I also tried setting `WSL_INTEROP` to the visible `/run/WSL/*_interop` sockets;
all failed the same way. So "run `usbipd.exe` from this SSH shell" is a good
goal, but it is not working right now.

## Options

### Option 1: Windows PowerShell Reset Script, Triggered Remotely

This is the recommended path.

Create a Windows-side script on g240, for example:

```powershell
# C:\Users\<you>\bin\Reset-ActivityBotUsbipd.ps1
$BusId = "1-6"
$Usbipd = "C:\Program Files\usbipd-win\usbipd.exe"

& $Usbipd detach --busid $BusId 2>$null
Start-Sleep -Milliseconds 750
& $Usbipd attach --wsl --busid $BusId
& $Usbipd list
```

Then enable a way to run that script remotely without sitting at g240:

```bash
ssh g240-windows 'powershell -ExecutionPolicy Bypass -File C:\Users\<you>\bin\Reset-ActivityBotUsbipd.ps1'
```

This likely means enabling Windows OpenSSH Server on g240, or using Tailscale
SSH for Windows if that is available. Microsoft documents that Windows can run
Linux commands with `wsl.exe`, and WSL can run Windows tools through interop;
here we are using the reverse direction through Windows SSH because WSL interop
is currently broken in the Codex SSH session
([Microsoft WSL interop docs](https://learn.microsoft.com/en-us/windows/wsl/filesystems)).

Daily workflow would become:

```bash
activitybot-usb-reset
timeout 12s sudo -n scripts/parallax-run-smoke.sh /dev/ttyUSB0
DISPLAY=:0 sg dialout -c 'cd /home/claude/sim2real && .venv/bin/python scripts/activitybot-encoder-dashboard-raylib.py --port /dev/ttyUSB0 --seconds 180'
```

Pros:

- No walking over to g240 for normal USB/IP resets.
- Keeps build, flash, serial, and raylib display in the WSL workflow.
- Uses the same `usbipd` recovery that is already verified.

Cons:

- Requires one-time setup of Windows remote PowerShell access.
- Need to secure the Windows SSH path reasonably.

### Option 2: Fix WSL-To-Windows Interop

The ideal command from WSL would be:

```bash
"/mnt/c/Program Files/usbipd-win/usbipd.exe" detach --busid 1-6
"/mnt/c/Program Files/usbipd-win/usbipd.exe" attach --wsl --busid 1-6
```

This matches Microsoft's documented WSL interop model, where Windows `.exe`
tools can be invoked from Linux
([Microsoft WSL interop docs](https://learn.microsoft.com/en-us/windows/wsl/filesystems)).

Current blocker:

```text
cmd.exe and usbipd.exe exist under /mnt/c, but launching them from this SSH
session fails with WSL interop socket errors.
```

Possible investigation paths:

- Start Codex from a normal interactive WSL terminal instead of a pure SSH
  session and see whether interop works there.
- Compare environment variables and `/run/WSL/*_interop` between a working local
  WSL terminal and this SSH session.
- Check whether WSL interop has been disabled or broken by the SSH/session
  launch path.

Pros:

- Cleanest long-term ergonomics if it works.
- No separate Windows SSH target needed.

Cons:

- It is not currently working.
- Debugging WSL interop could burn time unrelated to the robot.

### Option 3: Windows Watchdog Script

A Windows PowerShell loop could try to keep the ActivityBot attached:

```powershell
$BusId = "1-6"
$Usbipd = "C:\Program Files\usbipd-win\usbipd.exe"

while ($true) {
    $list = & $Usbipd list
    if ($list -notmatch "$BusId.*Attached") {
        & $Usbipd attach --wsl --busid $BusId
    }
    Start-Sleep -Seconds 5
}
```

This can be launched manually, from Task Scheduler at login, or as a scheduled
task. It is not a WSL cron job; it needs to run on Windows because `usbipd` is
Windows-side.

However, our observed failure can be subtler than "not attached." Windows may
still show the device attached while WSL's FTDI path is stale. A stronger
watchdog would need either:

- an unconditional detach/attach before each demo or flash, or
- a WSL health check that detects whether `propeller-load` can actually handshake
  with the chip.

Pros:

- Fully automated once running.
- Good for keeping the common "not attached" state fixed.

Cons:

- A dumb "is it attached?" watchdog may miss the stale-tty failure.
- A health-check watchdog that runs `propeller-load` can interfere with real
  flashing if not carefully coordinated.

### Option 4: Windows-Native Flashing Over COM Port

Another approach is to stop passing the USB adapter into WSL. Let Windows own
the COM port and drive flashing from Windows tools, then control those tools
remotely.

Pros:

- Avoids WSL USB/IP fragility.
- Windows serial support is the path SimpleIDE was originally designed around.

Cons:

- Splits the workflow: training/build/source in WSL, flashing on Windows.
- Need Windows-side Propeller loader setup and path translation.
- The raylib dashboard and Linux serial readers would need a different transport
  or Windows-side reader.

This is a fallback if `usbipd` remains too flaky.

### Option 5: WSL Cron Job

This is not recommended unless WSL interop is fixed.

WSL cron can run Linux commands, but the useful reset operation is Windows-side:

```powershell
usbipd detach --busid 1-6
usbipd attach --wsl --busid 1-6
```

Without working WSL-to-Windows interop, cron cannot reliably run those commands.
Even with interop fixed, cron blindly resetting USB while a flash or dashboard is
active would be dangerous. A per-command reset wrapper is safer.

## Recommended Near-Term Implementation

Do this in two pieces.

First, create a Windows script:

```powershell
# Reset-ActivityBotUsbipd.ps1
$BusId = "1-6"
$Usbipd = "C:\Program Files\usbipd-win\usbipd.exe"

Write-Host "Resetting ActivityBot USB/IP on BUSID $BusId"
& $Usbipd detach --busid $BusId 2>$null
Start-Sleep -Milliseconds 750
& $Usbipd attach --wsl --busid $BusId
& $Usbipd list
```

Second, create a Linux wrapper in this repo once Windows remote execution is
available:

```bash
#!/usr/bin/env bash
set -euo pipefail

ssh g240-windows 'powershell -ExecutionPolicy Bypass -File C:\Users\<you>\bin\Reset-ActivityBotUsbipd.ps1'
sleep 1
scripts/parallax-check.sh
```

Then make higher-level demo scripts call that wrapper before doing hardware
work:

```bash
scripts/activitybot-usb-reset.sh
timeout 12s sudo -n scripts/parallax-run-smoke.sh /dev/ttyUSB0
```

For now, the manual fallback remains:

```powershell
usbipd detach --busid 1-6
usbipd attach --wsl --busid 1-6
```

followed by:

```bash
timeout 12s sudo -n scripts/parallax-run-smoke.sh /dev/ttyUSB0
```

## Open Questions

- Can we enable Windows OpenSSH Server on g240 and reach it over Tailscale as
  `g240` or a separate Windows hostname?
- Does WSL interop work from a normal local WSL terminal on g240, even though it
  fails from this SSH-launched Codex session?
- Should the future reset wrapper always detach/attach before flash/demo, or
  only after detecting a failed handshake?
- Is `usbipd` stable enough for daily firmware work, or should Windows-native
  COM flashing become the fallback path?
