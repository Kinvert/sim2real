# ActivityBot Sim2Real Policy With PufferLib

Research notes for training a small line-following RL policy in PufferLib and
running it on a Parallax ActivityBot 360 / Propeller Activity Board WX.

This is not part of the affine environment work. It is a design/research note
for a future project.

## Bottom Line

The practical path is:

1. Train a tiny native PufferLib policy in a C Ocean environment.
2. Use PufferLib's native checkpoint format, which is a flat fp32 `.bin` for the
   native backend in this checkout, not ONNX and not a PyTorch runtime artifact
   ([pufferlib/pufferl.py](pufferlib/pufferl.py), [src/bindings.cu](src/bindings.cu),
   [src/puffernet.h](src/puffernet.h)).
3. Convert that flat weight file into either a compact C header or a raw SD-card
   binary with a tiny Propeller C inference function.
4. On the ActivityBot, read the three active QTI line sensors `[P7, P6, P5]`,
   run the policy at a modest control rate, and call
   `drive_speed(left_ticks_per_sec, right_ticks_per_sec)`
   from `abdrive360.h` ([ActivityBot 360 calibration / drive docs](https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/lessons/calibrate-the-activitybot-360/),
   [abdrive360.h source](https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h)).

Do not plan on importing ONNX, PyTorch, or an ONNX runtime on the ActivityBot.
The Propeller P8X32A has only 32 KB of hub RAM, while the Activity Board WX has
64 KB EEPROM and a microSD slot ([P8X32A datasheet, memory section](https://forums.parallax.com/uploads/attachments/41126/46295.pdf),
[Activity Board WX specs](https://www.parallax.com/product/propeller-activity-board-wx/)).
The model needs to be reduced to a few kilobytes and represented as plain C data
or a simple binary.

## Source Map

PufferLib sources used:

- PufferLib docs: native backend, Ocean C envs, PufferNet, build/train/eval
  commands ([PufferLib docs](https://puffer.ai/docs.html)).
- Current CLI and checkpoint flow
  ([pufferlib/pufferl.py](pufferlib/pufferl.py)).
- Native save/load of flat fp32 weights
  ([src/bindings.cu](src/bindings.cu)).
- Native policy construction and weight registration order
  ([src/pufferlib.cu](src/pufferlib.cu), [src/models.cu](src/models.cu)).
- Standalone C inference helpers and PufferNet model loader
  ([src/puffernet.h](src/puffernet.h)).
- C env binding pattern
  ([src/vecenv.h](src/vecenv.h), [ocean/minimal/binding.c](ocean/minimal/binding.c),
  [ocean/cartpole/binding.c](ocean/cartpole/binding.c)).
- Standalone policy demo pattern
  ([ocean/cartpole/cartpole.c](ocean/cartpole/cartpole.c)).
- PyTorch fallback save/load behavior
  ([pufferlib/torch_pufferl.py](pufferlib/torch_pufferl.py)).

Parallax sources used:

- Propeller C / SimpleIDE overview and C standard support
  ([Propeller C Reference](https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/),
  [SimpleIDE User Guide PDF](https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf)).
- ActivityBot 360 drive library behavior
  ([calibration / forward example](https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/lessons/calibrate-the-activitybot-360/),
  [speed control](https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/lessons/set-certain-speeds-2/),
  [abdrive360.h source](https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h)).
- ActivityBot 360 / Activity Board WX hardware
  ([ActivityBot 360 product page](https://www.parallax.com/product/activitybot-360-robot-kit/),
  [Propeller Activity Board WX product page](https://www.parallax.com/product/propeller-activity-board-wx/)).
- microSD file access from Propeller C
  ([SD Card Data tutorial](https://learn.parallax.com/courses/propeller-c-simple-devices/lessons/sd-card-data/)).
- Four-QTI line following behavior
  ([QTI Line Follower AppKit](https://www.parallax.com/product/qti-line-follower-appkit-for-the-small-robot/),
  [QTI build tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/build-the-qti-line-follower/),
  [QTI sensor test tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/),
  [QTI line-following program](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/program-for-line-following/),
  [QTI arrays version](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/line-following-using-arrays/),
  [QTI sensor product page](https://www.parallax.com/product/qti-sensor/)).

PyTorch / ONNX sources used:

- PyTorch `state_dict` save/load guidance
  ([PyTorch saving/loading models](https://docs.pytorch.org/tutorials/beginner/saving_loading_models.html)).
- PyTorch ONNX export overview
  ([torch.onnx docs](https://docs.pytorch.org/docs/stable/onnx.html)).

## Quick Parallax Links

- ActivityBot C tutorial collection:
  <https://learn.parallax.com/book_collection/activitybot-with-c-tutorials/>
- ActivityBot 360 Propeller C course:
  <https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/>
- Legacy ActivityBot Propeller C course:
  <https://learn.parallax.com/courses/legacy-version-propeller-c-programming-with-the-activitybot/>
- Propeller C reference:
  <https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/>
- Propeller C software / SimpleIDE downloads:
  <https://www.parallax.com/download/propeller-c-software/>
- SimpleIDE User Guide PDF:
  <https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf>
- SimpleIDE source:
  <https://github.com/parallaxinc/SimpleIDE>
- Simple Libraries source:
  <https://github.com/parallaxinc/Simple-Libraries>
- Propeller Activity Board WX:
  <https://www.parallax.com/product/propeller-activity-board-wx/>
- ActivityBot 360 Robot Kit:
  <https://www.parallax.com/product/activitybot-360-robot-kit/>
- QTI Line Follower AppKit:
  <https://www.parallax.com/product/qti-line-follower-appkit-for-the-small-robot/>
- QTI sensor:
  <https://www.parallax.com/product/qti-sensor/>
- SD card data tutorial:
  <https://learn.parallax.com/courses/propeller-c-simple-devices/lessons/sd-card-data/>
- `abdrive360.h` source:
  <https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h>

## What PufferLib Builds And Saves

PufferLib 4.0 has a native CUDA/C backend and a PyTorch fallback backend. The
docs describe the native backend as Python plus CUDA C, and Ocean environments as
plain C environments compiled into the training stack ([PufferLib docs](https://puffer.ai/docs.html)).
In this checkout, the `puffer` console script maps to `pufferlib.pufferl:main`,
and the supported modes in `main()` are `train`, `eval`, `sweep`, `paretosweep`,
and `match` ([pyproject.toml](pyproject.toml), [pufferlib/pufferl.py](pufferlib/pufferl.py)).

Native training saves checkpoints under:

```text
<checkpoint_dir>/<env_name>/<run_id>/<global_step>.bin
```

The native backend's `save_weights` copies `pufferl.master_weights` from device
to host and writes raw bytes to the target path. It writes `num_params *
sizeof(float)` bytes, with no JSON, protobuf, ONNX graph, or PyTorch pickle
container ([src/bindings.cu](src/bindings.cu)). The native `load_weights` checks
that file size exactly matches the expected number of fp32 bytes, reads the file,
copies it to device memory, and casts to bf16 parameters when the backend was
built in bf16 mode ([src/bindings.cu](src/bindings.cu)).

The PyTorch fallback path is different. `pufferlib/torch_pufferl.py` saves
`policy.state_dict()` via `torch.save`, and loads it with `torch.load` plus
`load_state_dict` ([pufferlib/torch_pufferl.py](pufferlib/torch_pufferl.py)).
PyTorch's own docs describe `state_dict` saving as the recommended PyTorch
method for restoring a model in Python ([PyTorch saving/loading models](https://docs.pytorch.org/tutorials/beginner/saving_loading_models.html)).
Because PufferLib still names the path `*.bin`, the extension alone is not enough
to know the format: native `.bin` is raw fp32; `--slowly` PyTorch `.bin` is a
PyTorch serialization ([pufferlib/pufferl.py](pufferlib/pufferl.py),
[pufferlib/torch_pufferl.py](pufferlib/torch_pufferl.py)).

## Native Model Architecture

The native PufferLib policy in this checkout is:

```text
observation -> encoder linear -> MinGRU layers -> decoder linear -> logits + value
```

The default native encoder is one matrix multiply from `input_size` to
`hidden_size`, with no bias ([src/models.cu](src/models.cu)). The decoder is one
matrix multiply from `hidden_size` to `output_dim + 1`, where the extra output is
the value head; for continuous actions it also has a `logstd` tensor
([src/models.cu](src/models.cu)). The network between encoder and decoder is
MinGRU, with each layer having a bias-free `(3 * hidden_size) x hidden_size`
projection ([src/models.cu](src/models.cu)).

The standalone C inference helper `src/puffernet.h` mirrors this native model.
Its comment states the PufferNet weight order:

```text
encoder weight: hidden_dim x input_dim
decoder weight: (atn_sum + 1) x hidden_dim, last output is value
decoder logstd: only for continuous policies
mingru weights[0..num_layers-1]: (3 * hidden_dim) x hidden_dim each
```

That same helper provides `load_weights`, `make_puffernet`, and
`forward_puffernet` for standalone C demos ([src/puffernet.h](src/puffernet.h)).
Existing demos load a `resources/.../*_weights.bin` file, create `PufferNet`,
and call `forward_puffernet` before stepping the env, e.g. cartpole and drive
([ocean/cartpole/cartpole.c](ocean/cartpole/cartpole.c),
[ocean/drive/drive.c](ocean/drive/drive.c)).

For deployment, the decoder's value output is not needed. The ActivityBot only
needs the action logits. For deterministic real-time control, use argmax over
the action logits instead of stochastic softmax sampling. PufferLib's standalone
helper currently samples discrete actions via `softmax_multidiscrete`, which
uses `expf` and `rand()` ([src/puffernet.h](src/puffernet.h)). On the Propeller,
argmax is simpler, deterministic, and avoids `expf`.

## Expected Model Size

For a discrete line follower with four observation values and one action head,
the native PufferNet weight count is approximately:

```text
encoder = H * obs_size
decoder = (num_actions + 1) * H
mingru  = num_layers * 3 * H * H
```

The native C helper aligns tensors for bf16-native checkpoint compatibility, so
using a hidden size divisible by 8 is the least surprising path
([src/puffernet.h](src/puffernet.h), [src/kernels.cu](src/kernels.cu)).

Example sizes for `obs_size = 4`, `num_actions = 7`, `num_layers = 1`:

| Hidden | Floats | fp32 bytes | int8 bytes |
| ---: | ---: | ---: | ---: |
| 8 | `32 + 64 + 192 = 288` | 1,152 | 288 |
| 16 | `64 + 128 + 768 = 960` | 3,840 | 960 |
| 32 | `128 + 256 + 3,072 = 3,456` | 13,824 | 3,456 |
| 64 | `256 + 512 + 12,288 = 13,056` | 52,224 | 13,056 |

The Propeller P8X32A hub RAM is 32 KB, and C code, stack, heap, libraries, servo
control, SD buffers, policy state, activations, and weights all compete for that
space ([P8X32A datasheet](https://forums.parallax.com/uploads/attachments/41126/46295.pdf),
[SimpleIDE User Guide](https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf)).
A hidden size of 8 or 16 is the reasonable first target. Hidden 32 might fit only
with careful static allocation and a stripped inference path. Hidden 64 is too
large for a simple Propeller 1 deployment unless weights stay external and code
is heavily optimized.

## Why ONNX Is Not The ActivityBot Import Path

PyTorch can export a `torch.nn.Module` to ONNX, and the PyTorch ONNX docs describe
`torch.onnx.export` as converting a PyTorch module computation graph into an
ONNX graph consumable by ONNX runtimes ([torch.onnx docs](https://docs.pytorch.org/docs/stable/onnx.html)).
That is useful on PCs and embedded Linux systems with a runtime.

The ActivityBot / Propeller 1 target is different:

- Propeller C is compiled by PropellerGCC for the P8X32A, not a hosted OS with
  dynamic libraries ([Propeller C Reference](https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/)).
- The P8X32A has 32 KB hub RAM, which is far below what an ONNX parser/runtime
  expects ([P8X32A datasheet](https://forums.parallax.com/uploads/attachments/41126/46295.pdf)).
- Parallax explicitly warns that standard libraries such as `stdio.h` are large,
  and provides smaller Simple Libraries for microcontroller work
  ([Propeller C Reference](https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/)).
- The exact policy architecture is known and tiny, so a handwritten forward pass
  is much smaller than a general ML runtime ([src/puffernet.h](src/puffernet.h)).

ONNX can still be useful as an offline inspection/interchange format if we use
the PyTorch backend for experiments. It should not be the robot-side policy
format. The robot-side artifact should be a tiny architecture-specific C header
or binary file.

## How PufferLib C Environments Work

PufferLib's docs say Ocean environments are written in C, with an `.h` file for
core environment logic and a `.c` file for a standalone demo; only the `.h` is
compiled for training, while standalone builds use local/fast/web options
([PufferLib docs](https://puffer.ai/docs.html)). The docs also state that
observations, actions, rewards, and terminals are allocated as contiguous chunks
across many environment instances ([PufferLib docs](https://puffer.ai/docs.html)).

In this checkout, a C env binding defines:

- `OBS_SIZE`
- `NUM_ATNS`
- `ACT_SIZES`
- `OBS_TENSOR_T`
- `Env`
- `my_init`
- `my_log`

Then it includes `vecenv.h` ([ocean/minimal/binding.c](ocean/minimal/binding.c),
[ocean/cartpole/binding.c](ocean/cartpole/binding.c), [src/vecenv.h](src/vecenv.h)).
The core env struct must expose observations, actions, rewards, terminals, log,
agent count, and RNG fields in the expected shape ([ocean/minimal/minimal.h](ocean/minimal/minimal.h),
[ocean/cartpole/cartpole.h](ocean/cartpole/cartpole.h)).

For a line follower, this maps naturally:

```c
#define OBS_SIZE 3
#define NUM_ATNS 2
#define ACT_SIZES {1, 1}
#define OBS_TENSOR_T FloatTensor
```

The current policy observations are three normalized QTI RC-decay readings:
`[left, middle, right]`. Firmware reads them from `[P7, P6, P5]`; P4/far-right
is intentionally ignored in this branch. Digital QTI states are still useful for
debugging and heuristic baselines, but they are a thresholded view of the
underlying sensor, not the only real signal available. The two continuous
actions map to left/right wheel speed commands.

## Proposed Line-Follower Environment

### Sim State

Use a 2D differential-drive simulation:

- Track: centerline spline or polyline with configurable tape width.
- Robot: pose `(x, y, theta)`, left/right wheel velocities, wheelbase, wheel
  radius, max ticks per second.
- Sensors: three sample points fixed to the robot body, ordered
  `[left, middle, right]`, with lateral offsets `[+20 mm, 0 mm, -20 mm]`.
- Sensor output: local reflectance mapped to a capacitor decay time, then
  normalized to `[0, 1]` using per-sensor white/black calibration values.
- Policy input: `[left, middle, right]` for the current small deployment model.

This matches the QTI kit's real setup: Parallax's QTI Line Follower AppKit mounts
three or four QTI sensors under the robot chassis, and Parallax documents the
QTI as a Charge Transfer Infrared sensor whose capacitor decay time depends on
reflected infrared light. Parallax also documents a fixed-timing digital mode
for black/white line following
([QTI Line Follower AppKit](https://www.parallax.com/product/qti-line-follower-appkit-for-the-small-robot/),
[QTI build tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/build-the-qti-line-follower/),
[QTI product page](https://www.parallax.com/product/qti-sensor/),
[QTI KickStart](https://learn.parallax.com/kickstarts/qti-sensor/)).

### Observations

The hardware scalar sensor interface is normalized in physical order and passed
directly to the policy:

```text
obs[0] = left_qti_darkness    (P7, +20 mm lateral)
obs[1] = middle_qti_darkness  (P6,   0 mm lateral)
obs[2] = right_qti_darkness   (P5, -20 mm lateral)
```

Polarity should be:

```text
0.0 ~= calibrated white / high reflection / lower decay time
1.0 ~= calibrated black / low reflection / higher decay time
```

Parallax's KickStart page says lower numeric readings correspond to lighter
surfaces and higher numeric readings correspond to darker surfaces, and its
Propeller examples report RC-time values as numbers
([QTI KickStart](https://learn.parallax.com/kickstarts/qti-sensor/)).
Propeller C's `rc_time(pin, 1)` path is documented in the Sense Light lesson:
the program charges the pin high, waits briefly, then measures decay time past
the Propeller logic threshold
([Propeller C Sense Light](https://learn.parallax.com/courses/propeller-c-simple-circuits/lessons/sense-light/)).

The digital bit view should use the same polarity as Parallax's Blockly QTI
line-following tutorial: black line returns `1`, white returns `0`
([QTI sensor test tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/)).
Using the same polarity for debug bits reduces robot-side translation mistakes.

If the four scalar readings are too partially observable, add one or two values that
can also be maintained on the robot:

- previous action index
- time since any sensor saw black
- previous thresholded sensor mask

Do not add privileged sim-only state such as true lateral error unless the goal
is teacher imitation or reward shaping only. A deployed RL policy can only use
what the ActivityBot can read.

### Actions

Use one discrete action head. A seven-action table is enough:

| Action | Meaning | Left tps | Right tps |
| ---: | --- | ---: | ---: |
| 0 | hard left / pivot left | 16 | 64 |
| 1 | left | 32 | 64 |
| 2 | slight left | 56 | 64 |
| 3 | forward | 64 | 64 |
| 4 | slight right | 64 | 56 |
| 5 | right | 64 | 32 |
| 6 | hard right / pivot right | 64 | 16 |

Parallax's own four-QTI line-following table uses the same idea: outer sensors
drive pivot/sharp turns, inner sensors drive slighter corrections, and centered
inner readings drive forward behavior ([QTI line-following program](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/program-for-line-following/),
[QTI arrays version](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/line-following-using-arrays/)).
The actual tps values should be randomized in sim and tuned on hardware.

### Reward

A first reward can be:

```text
reward =
  + forward_progress_along_track
  - 0.2 * abs(lateral_error / tape_width)
  - 0.05 * abs(turn_rate)
  - 0.10 * action_changed
  - 1.0 if all sensors white for N consecutive steps
```

Terminate or reset after the robot has lost the line for a short window, after
leaving the course bounds, or after a max episode time. PufferLib docs call out
that environments should handle resets internally, and that observation/reward
scale should stay near `-1..1` ([PufferLib docs](https://puffer.ai/docs.html)).

### Domain Randomization

Randomize these during training:

- tape width and curve radius
- robot starting lateral offset and heading error
- wheelbase, wheel radius, and left/right motor gain
- battery speed scaling
- sensor spacing, sensor threshold, sensor latency, and bit-flip noise
- friction / slip on turns
- control loop period
- occasional missing tape segments

The real QTI sensor depends on reflected infrared and capacitor decay time, so
height, reflectivity, room light, and surface material can change readings
([QTI sensor product page](https://www.parallax.com/product/qti-sensor/),
[QTI sensor test tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/)).
Randomization should cover those variations before any real-world run.

## Parallax ActivityBot / Propeller C Constraints

### Hardware

The ActivityBot 360 uses the Propeller Activity Board WX and the Propeller
P8X32A-Q44 microcontroller ([ActivityBot 360 product page](https://www.parallax.com/product/activitybot-360-robot-kit/),
[Activity Board WX specs](https://www.parallax.com/product/propeller-activity-board-wx/)).
The Activity Board WX product page lists:

- Propeller P8X32A-Q44
- 64 KB I2C EEPROM
- microSD card slot
- 12-bit 4-channel SPI ADC
- servo headers
- USB serial programming/debugging

The P8X32A has eight cogs, all sharing hub memory through a round-robin hub
([P8X32A intro](https://www.parallax.com/propeller/qna/Content/QnaTopics/QnaP8X32AIntro.htm),
[Propeller 1 overview](https://www.parallax.com/propeller-1/)).
The datasheet states main memory is 64 KB total, made of 32 KB RAM and 32 KB ROM;
the 32 KB RAM is where the application is loaded from host or EEPROM
([P8X32A datasheet](https://forums.parallax.com/uploads/attachments/41126/46295.pdf)).

### C / IDE

Propeller C uses PropellerGCC. Parallax states that PropellerGCC is an
open-source C/C++ compiler for P8X32A and is ANSI C89/C99 compliant with very few
exceptions ([Propeller C Reference](https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/)).
I did not find a Parallax source claiming C11 support for the ActivityBot
Propeller C stack, so C99 should be treated as the practical ceiling for this
target ([Propeller C Reference](https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/)).
SimpleIDE is the C programming environment for Propeller C tutorials, and the
SimpleIDE User Guide says it supports C, C++, Spin, and Propeller Assembly and
ships with PropGCC and OpenSpin ([SimpleIDE User Guide PDF](https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf)).

Parallax also says some standard libraries such as `stdio.h` are large, and that
the Propeller C Learning System includes smaller Simple Libraries for
microcontroller I/O tasks ([Propeller C Reference](https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/)).
That matters for policy deployment: avoid general parsers, formatted file I/O,
dynamic allocation, and heavyweight math where a lookup table or fixed-point
integer code will do.

The `abdrive360.h` header itself says it should be used with CMM, uses one
additional core, and reads calibration data from EEPROM
([abdrive360.h source](https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h)).

### Drive Commands

For ActivityBot 360 programs, Parallax shows:

```c
#include "simpletools.h"
#include "abdrive360.h"

drive_speed(64, 64);
pause(4000);
drive_speed(0, 0);
```

The docs state that `abdrive360.h` is required for ActivityBot 360 and that
`abdrive.h` will not work for it ([ActivityBot 360 calibration / forward example](https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/lessons/calibrate-the-activitybot-360/)).
`drive_speed(left, right)` sets wheel speed in encoder ticks per second, and
positive values are forward while negative values are reverse ([set certain speeds](https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/lessons/set-certain-speeds-2/),
[abdrive360.h source](https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h)).
The ActivityBot docs define 64 ticks as one wheel revolution, and `abdrive360.h`
documents maximum speed as +/-128 ticks/sec and one tick as about 3.25 mm
([ActivityBot 360 calibration / forward example](https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/lessons/calibrate-the-activitybot-360/),
[abdrive360.h source](https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h)).
`drive_goto(left_ticks, right_ticks)` blocks by default until a distance move is
complete, and `drive_getTicks(&left, &right)` reads measured ticks since program
start ([abdrive360.h source](https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h)).

For line following, `drive_speed` is the right command. `drive_goto` is useful
for calibration tests but not for a continuous sensor-feedback loop.

### QTI Sensors

The QTI Line Follower AppKit includes four QTI sensors and is explicitly sold for
ActivityBot 360 line-following ([QTI Line Follower AppKit](https://www.parallax.com/product/qti-line-follower-appkit-for-the-small-robot/)).
Parallax's QTI tutorial mounts the sensors under the chassis and wires each
sensor to 5 V, a Propeller I/O pin, and ground ([QTI build tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/build-the-qti-line-follower/)).
The QTI sensor product page says the sensor can be used digitally for fast
black/white line following or as an analog sensor for shades of gray
([QTI sensor product page](https://www.parallax.com/product/qti-sensor/)).
It also says the microcontroller measures how long the QTI capacitor takes to
decay, which is a rate of charge transfer through an infrared phototransistor
and therefore indicates how much IR reflects from the nearby surface. The
KickStart page gives the deployment-significant polarity: lower numbers mean
lighter/more reflective or closer, higher numbers mean darker/less reflective
or farther, with typical values in the tens-to-thousands range depending on the
microcontroller and surface
([QTI product page](https://www.parallax.com/product/qti-sensor/),
[QTI KickStart](https://learn.parallax.com/kickstarts/qti-sensor/)).

The BlocklyProp QTI tutorial describes the digital read cycle: set the four I/O
states high/output, flash for about 230 us, switch to input for about 230 us, and
read the pins. It states black returns `1` and white returns `0`
([QTI sensor test tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/)).
For Propeller C, the first raw-read prototype should use `simpletools`
`rc_time(pin, 1)` after charging the pin high, matching Parallax's Sense Light
pattern. The Blockly-style 230 us fixed-timing read is still useful as a
digital fallback and sanity check
([Propeller C Sense Light](https://learn.parallax.com/courses/propeller-c-simple-circuits/lessons/sense-light/)).

### microSD

The ActivityBot 360 product page says the kit has built-in SD-card readiness for
data logging and file storage ([ActivityBot 360 product page](https://www.parallax.com/product/activitybot-360-robot-kit/)).
The Activity Board WX product page lists a microSD card slot ([Activity Board WX specs](https://www.parallax.com/product/propeller-activity-board-wx/)).
The Propeller Activity Board guide maps the SD pins as P22 DO, P23 CLK, P24 DI,
and P25 CS ([Activity Board guide PDF](https://learn.parallax.com/sites/default/files/content/propeller-c-tutorials/PropellerAB/32910-Propeller-Activity-Board-Guide-v1.0.pdf)).

Parallax's SD Card Data tutorial uses:

```c
int DO = 22, CLK = 23, DI = 24, CS = 25;
sd_mount(DO, CLK, DI, CS);
FILE* fp = fopen("test.txt", "r");
fread(buffer, 1, n, fp);
fclose(fp);
```

The same tutorial says `fopen`, `fread`, `fwrite`, and `fclose` are available
through `stdio.h` with Propeller GCC, but warns that `fprintf` and `fscanf` use
far more memory than `fwrite`/`fread` ([SD Card Data tutorial](https://learn.parallax.com/courses/propeller-c-simple-devices/lessons/sd-card-data/)).
So the policy file should be a compact binary read with `fread`, not text or CSV.

## Robot-Side Policy Format

Recommended first implementation:

```text
policy.h
  #define OBS_SIZE 3
  #define HIDDEN_SIZE 4
  #define NUM_ACTIONS 2
  static const int8_t encoder_w[HIDDEN_SIZE][OBS_SIZE] = ...
  static const int8_t decoder_w[NUM_ACTIONS + 1][HIDDEN_SIZE] = ...
  static const int8_t mingru_w[3 * HIDDEN_SIZE][HIDDEN_SIZE] = ...
  static const int scale/... = ...
```

Then write fixed-point inference in C:

```text
read 3 QTI RC-time values
normalize to calibrated 0.0..1.0 fixed-point
encoder matmul
MinGRU update if NUM_LAYERS > 0
decoder matmul to continuous left/right actions
clamp/scale actions to left/right wheel speeds
drive_speed(left_ticks_per_second, right_ticks_per_second)
```

If the C header makes the compiled program too large, store the same arrays in a
binary file on SD and read them once at startup using `sd_mount` + `fread`
([SD Card Data tutorial](https://learn.parallax.com/courses/propeller-c-simple-devices/lessons/sd-card-data/)).
The Activity Board WX has both 64 KB EEPROM and microSD, but the EEPROM is also
used for program storage and ActivityBot calibration, so SD is safer for model
iteration ([Activity Board WX specs](https://www.parallax.com/product/propeller-activity-board-wx/),
[abdrive360.h source](https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h)).

For the first hardware smoke test, fp32 may be acceptable with `H=8` at a low
control rate. For real line-following, int8 or int16 fixed-point is the safer
target because Propeller 1 has no FPU and has only 32 KB hub RAM
([P8X32A datasheet](https://forums.parallax.com/uploads/attachments/41126/46295.pdf),
[Propeller C Reference](https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/)).

## Conversion Pipeline

1. Add a PufferLib Ocean env, probably `ocean/activitybot_line/`.
   Follow the existing binding pattern from minimal/cartpole:
   `activitybot_line.h`, `binding.c`, optional standalone demo `.c`, and
   `config/activitybot_line.ini` ([ocean/minimal/binding.c](ocean/minimal/binding.c),
   [ocean/cartpole/binding.c](ocean/cartpole/binding.c), [PufferLib docs](https://puffer.ai/docs.html)).

2. Train native, not `--slowly`, so the checkpoint is raw fp32:

   ```bash
   bash build.sh activitybot_line --float
   puffer train activitybot_line --policy.hidden-size 8 --policy.num-layers 1
   ```

   The docs show `bash build.sh ENV_NAME`, `puffer train ENV_NAME`, and
   `puffer eval ENV_NAME --load-model-path latest` as the normal flow
   ([PufferLib docs](https://puffer.ai/docs.html)).

3. Verify in a standalone C demo using `src/puffernet.h` and
   `forward_puffernet`, matching how cartpole and drive demos load resources
   ([src/puffernet.h](src/puffernet.h), [ocean/cartpole/cartpole.c](ocean/cartpole/cartpole.c),
   [ocean/drive/drive.c](ocean/drive/drive.c)).

4. Convert checkpoint to a Propeller-friendly format:

   - Read raw fp32 in Python on the PC.
   - Slice weights according to architecture.
   - Quantize to int8 or int16 per tensor.
   - Emit `policy_weights.h` or `POLICY.BIN`.
   - Emit a tiny metadata header with magic/version, obs size, action count,
     hidden size, layer count, quantization scales, and checksum.

5. Build ActivityBot C:

   - Include `simpletools.h` and `abdrive360.h`
     ([ActivityBot 360 calibration / forward example](https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/lessons/calibrate-the-activitybot-360/)).
   - Initialize/calibrate QTI thresholds.
   - Load policy from C constants or SD.
   - Loop at a fixed rate.
   - Run argmax inference.
   - Call `drive_speed`.

6. Validate sim-to-real:

   - First run on a stand with wheels off the ground.
   - Print sensor mask, action, left/right command over serial.
   - Run slowly on a simple oval track.
   - Increase speed only after sensor polarity and action direction are verified.

## Inference Details For Propeller C

The current C helper's MinGRU uses:

```text
combined = W @ x
hidden, gate, highway = split(combined)
gate_s = sigmoid(gate)
h_tilde = hidden + 0.5 if hidden >= 0 else sigmoid(hidden)
mingru_out = state + gate_s * (h_tilde - state)
hw_s = sigmoid(highway)
output = hw_s * mingru_out + (1 - hw_s) * x
state = mingru_out
```

This comes directly from `mingru()` in `src/puffernet.h`, which is the
standalone inference mirror of the native MinGRU kernels
([src/puffernet.h](src/puffernet.h), [src/models.cu](src/models.cu)).

For Propeller 1, the expensive operations are matrix multiplies and sigmoid.
With `H=8` or `H=16`, matrix multiplies are small. Sigmoid can be implemented as:

- a lookup table over a clipped fixed-point range,
- a hard-sigmoid approximation,
- or fp32 `expf` for the first slow prototype.

Do not use softmax for the deployed discrete policy. Argmax over logits gives
the same greedy action without `expf`, random sampling, or probability
normalization ([src/puffernet.h](src/puffernet.h)).

## Open Technical Risks

- **PufferNet recurrence may be unnecessary.** A line follower with only four
  digital sensors is partly observable when all sensors are white, but previous
  action and previous mask may be enough. Native PufferNet assumes MinGRU; if we
  want a pure feed-forward policy, we should either add a native deployment path
  or use a tiny custom PyTorch/offline trainer. The current native/standalone
  path is simplest with `num_layers = 1` ([src/pufferlib.cu](src/pufferlib.cu),
  [src/puffernet.h](src/puffernet.h)).

- **Software float may be too slow.** The Propeller 1 target has no hardware FPU
  and only 32 KB hub RAM, so fixed-point or lookup-table sigmoid is likely needed
  after the first proof of concept ([P8X32A datasheet](https://forums.parallax.com/uploads/attachments/41126/46295.pdf)).

- **Compiled size matters.** `abdrive360.h` documents that it uses an additional
  core and CMM, and Parallax warns that standard libraries can be large
  ([abdrive360.h source](https://raw.githubusercontent.com/parallaxinc/Simple-Libraries/master/Learn/Simple%20Libraries/Robotics/ActivityBot360/libabdrive360/abdrive360.h),
  [Propeller C Reference](https://learn.parallax.com/propeller_c_referenc/propeller-c-reference/)).
  Every extra parser or formatted I/O call makes fitting harder.

- **QTI polarity and timing must match.** Raw RC-time readings should normalize
  lighter/whiter readings toward `0.0` and darker/blacker readings toward `1.0`
  ([QTI KickStart](https://learn.parallax.com/kickstarts/qti-sensor/)).
  The optional digital fallback should match Parallax's BlocklyProp polarity:
  black `1`, white `0`
  ([QTI sensor test tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/)).
  A reversed sensor order or polarity will make a good policy look broken.

- **Sim2real depends more on sensor/motor randomization than RL algorithm.** The
  robot has a simple dynamics/control interface, but tape width, sensor height,
  wheel slip, battery voltage, and light conditions strongly affect behavior
  ([QTI sensor product page](https://www.parallax.com/product/qti-sensor/),
  [ActivityBot speed docs](https://learn.parallax.com/courses/propeller-c-programming-with-the-activitybot-360/lessons/set-certain-speeds-2/)).

## Recommended Next Work

1. Build `activitybot_line` as a minimal Ocean C env with `OBS_SIZE=4`,
   `ACT_SIZES={7}`, and a standalone renderer.
2. Train `H=8, num_layers=1` first; only increase hidden size if the policy fails
   after reward and randomization fixes.
3. Add a PC-side checker that loads a native `.bin`, runs `src/puffernet.h`
   inference against a Python reference, and verifies argmax equality.
4. Write a converter from native `.bin` to `policy_weights.h` and `POLICY.BIN`.
5. Write a Propeller C prototype with fake hardcoded sensor masks before wiring
   real QTI sensors.
6. Move to QTI input, then wheels-on-stand, then slow track tests.

## Host And Flash Workflow For g240

Current topology:

```text
5090 Ubuntu workstation
  -> SSH over Tailscale/LAN
  -> g240 Windows 11 host
  -> WSL2 Ubuntu session where PufferLib training runs
  -> ActivityBot USB mini-B cable attached to g240
```

The best long-term setup is to keep training, sim, sweep scripts, model
conversion, and most firmware source in g240's WSL filesystem. Microsoft
recommends keeping files in the WSL/Linux filesystem for best Linux command-line
performance, and using Windows paths only when Windows tooling is the main user
of the files ([WSL filesystem guidance](https://learn.microsoft.com/en-us/windows/wsl/filesystems)).
That means the RL repo should live somewhere like:

```text
~/src/activitybot-sim2real/
  puffer_env/       # PufferLib/Ocean env work
  firmware/         # Propeller C source, Makefile, policy header/bin
  tools/            # checkpoint -> firmware artifact conversion
  scripts/          # train, export, flash, serial monitor wrappers
```

Do not make SimpleIDE the source of truth. Use it as a known-good reference and
emergency GUI if needed, but make the robot firmware buildable and flashable from
scripts. Parallax's SimpleIDE User Guide says the Build Status pane shows command
strings passed to the compiler and loader, so one useful first step is to build a
minimal blink/drive program once in SimpleIDE and copy the exact compiler/loader
commands into a Makefile or script ([SimpleIDE User Guide](https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf)).

### Option A: Flash From WSL2 With USB Attached To WSL

This is the cleanest workflow for SSH and Codex:

```text
Codex/SSH in WSL -> Linux Propeller compiler/loader -> /dev/ttyUSB0 -> robot
```

Microsoft says WSL2 does not have native USB attachment by default; the supported
path is `usbipd-win`, which can attach USB devices to a WSL2 distribution
([Microsoft WSL USB docs](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)).
The same Microsoft doc specifically names developer scenarios such as flashing
Arduino-style boards as a use case for `usbipd-win`
([Microsoft WSL USB docs](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)).

One-time Windows host setup on g240:

```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list
usbipd bind --busid <busid>
```

`usbipd bind` requires administrator privileges, while `usbipd attach --wsl` can
then be run without an elevated prompt according to Microsoft's instructions
([Microsoft WSL USB docs](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)).
Attach before a flash session:

```powershell
usbipd attach --wsl --busid <busid>
```

Then verify in WSL:

```bash
lsusb
dmesg | tail
ls -l /dev/ttyUSB* /dev/ttyACM*
```

The Microsoft doc notes that while a USB device is attached to WSL, Windows
cannot use it, and that WSL may need udev/permission setup for non-root access
([Microsoft WSL USB docs](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)).
For a first pass, using `sudo` for the loader is acceptable. Later, add a udev
rule or group membership if the device appears with restrictive permissions.

For tools, there are two realistic Linux choices:

- Install Parallax's official Linux SimpleIDE package. Parallax says the Linux
  package is official Propeller C tutorial software and supports Debian, Mint,
  and Ubuntu Linux on Intel 32/64-bit, with dependencies including `libftdi1`
  ([SimpleIDE Linux package](https://www.parallax.com/package/simpleide-software-for-linux-propeller-c/)).
- Build/install command-line tools directly. Parallax's PropGCC repository is a
  GCC port to the Propeller and includes `propeller-load`, and its README says
  output installs to `/opt/parallax` by default unless `PREFIX` is changed
  ([PropGCC README](https://github.com/parallaxinc/propgcc)). Parallax's newer
  PropLoader supports serial and WiFi downloads, can list ports, select a serial
  port, program EEPROM, run after download, enter terminal mode, and write files
  to SD (`-P`, `-p`, `-e`, `-r`, `-t`, `-f`)
  ([PropLoader README](https://raw.githubusercontent.com/parallaxinc/PropLoader/master/README.md)).

Expected flash script shape:

```bash
#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-/dev/ttyUSB0}"
IMAGE="build/activitybot.binary"

proploader -P
proploader -p "$PORT" -b activityboard -e -r "$IMAGE"
```

The exact board name should be copied from SimpleIDE's board config or discovered
from the loader's help/board list; do not guess it for the final script.
SimpleIDE's guide says board configuration files customize Propeller GCC hardware
and that Propeller GCC programs are loaded with `propeller-load`
([SimpleIDE User Guide](https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf)).

For policy files, PropLoader's `-f` SD-card option is potentially useful:

```bash
proploader -p "$PORT" -b activityboard -f POLICY.BIN
```

The Activity Board WX has a microSD holder, and Parallax documents the Propeller
Activity Board SD pins as P22 DO, P23 CLK, P24 DI, and P25 CS
([Activity Board guide PDF](https://learn.parallax.com/sites/default/files/content/propeller-c-tutorials/PropellerAB/32910-Propeller-Activity-Board-Guide-v1.0.pdf)).
PropLoader's README says `-f` writes a file to the SD card
([PropLoader README](https://raw.githubusercontent.com/parallaxinc/PropLoader/master/README.md)).
This can avoid repeatedly removing the microSD card once the loader path is
working.

### Option B: Flash From Windows Host, Drive It Remotely

This is the most conservative USB path:

```text
Codex/SSH in WSL or Windows SSH -> Windows loader/SimpleIDE tools -> COMx -> robot
```

Parallax's Windows SimpleIDE package is the official Propeller C tutorial
programming software for Windows and includes the Propeller GCC toolchain
([SimpleIDE Windows package](https://www.parallax.com/package/simpleide-software-for-windows-propeller-c/)).
SimpleIDE's user guide describes `Run with Terminal`, `Load RAM & Run`, and
`Load EEPROM & Run`, and says to pick a port whose type is "USB Serial" or
"FT232" ([SimpleIDE User Guide](https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf)).
The Activity Board WX product page lists USB mini-B onboard serial over USB as
one of its communication methods ([Activity Board WX product page](https://www.parallax.com/product/propeller-activity-board-wx/)).

In this mode, Windows owns the USB COM port and WSL does not need `usbipd`.
Codex can still work from WSL if Windows interop is working, because Microsoft
documents that WSL can run Windows tools directly from the Linux command line by
calling `[tool-name].exe` ([WSL interop docs](https://learn.microsoft.com/en-us/windows/wsl/filesystems)).
A wrapper might eventually look like:

```bash
WIN_LOADER="/mnt/c/Program Files (x86)/Parallax Inc/SimpleIDE/bin/proploader.exe"
"$WIN_LOADER" -P
"$WIN_LOADER" -p COM5 -b activityboard -e -r "$(wslpath -w build/activitybot.binary)"
```

However, in this current g240 WSL SSH session, direct Windows executable
interop is not currently usable: `/mnt/c/Windows/System32/cmd.exe` and
`powershell.exe` exist, but attempting to run them fails with WSL interop socket
errors. So if we choose the Windows-host path, setup should include either fixing
WSL interop for this SSH session or enabling Windows OpenSSH on g240 so the 5090
can SSH into Windows PowerShell directly. The Microsoft docs support both
directions: WSL can run Windows tools, and Windows can run Linux commands via
`wsl.exe` ([WSL interop docs](https://learn.microsoft.com/en-us/windows/wsl/filesystems),
[WSL basic commands](https://learn.microsoft.com/en-us/windows/wsl/basic-commands)).

Use Windows SimpleIDE GUI only for initial bring-up or when the command-line
loader path is broken. It is not the preferred daily workflow from the 5090
because GUI work over a side monitor does not compose well with SSH, automation,
or Codex.

### Option C: WiFi Programming Later

Do not make WiFi programming the first path, but keep it in mind. The Activity
Board WX has a wireless module socket with WiFi support, but the RF module is
not included on the board ([Activity Board WX product page](https://www.parallax.com/product/propeller-activity-board-wx/)).
Parallax's WX WiFi tutorial says the Activity Board WX can route program,
terminal, and application data through a WX WiFi module depending on the socket
configuration ([WX WiFi Prop C tutorial](https://learn.parallax.com/courses/parallax-wx-wi-fi-module-for-prop-c/lessons/connect-wx-wi-fi-module-to-your-propeller/)).
PropLoader also supports loading over serial or a WiFi connection to a Parallax
WiFi module on the Propeller Activity Board WX ([PropLoader README](https://raw.githubusercontent.com/parallaxinc/PropLoader/master/README.md)).

This could eventually be the best "Codex can flash the robot over Tailscale"
story, but it adds module firmware, network, and boot/config variables. Use USB
first.

### What Codex Agents Need

To let Codex safely build and flash, make hardware access scriptable:

```text
firmware/Makefile
scripts/build_firmware.sh
scripts/flash_activitybot.sh --port /dev/ttyUSB0 --eeprom
scripts/serial_activitybot.sh --port /dev/ttyUSB0 --baud 115200
scripts/install_policy_sd.sh --port /dev/ttyUSB0 POLICY.BIN
```

The flash script should print the exact port, board type, image path, and whether
it is writing RAM or EEPROM before doing anything. It should require an explicit
`--yes` flag for EEPROM writes. SimpleIDE distinguishes RAM load from EEPROM load
in its Program menu, and EEPROM writes persist through reset/power cycle
([SimpleIDE User Guide](https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf)).

For safety, the standard hardware checklist should be:

```text
1. robot on stand or wheels removed
2. battery voltage reasonable
3. QTI harness connected
4. port confirmed by proploader -P or usbipd/lsusb
5. flash to RAM first
6. serial prints show sensor mask and action
7. only then write EEPROM
```

Given the current machines, the recommended first implementation is Option A:
install the Propeller command-line toolchain in g240 WSL, attach the ActivityBot
USB serial device to WSL with `usbipd-win`, and drive build/flash/serial from the
same SSH session used for PufferLib. Keep Option B as a fallback because Windows
USB serial support is the path SimpleIDE was designed around.

## Verified g240 ActivityBot Bring-Up

The first working setup on g240 used the official SimpleIDE Linux package
extracted locally under `tools/parallax/simpleide` instead of installed into
`/opt/parallax`. This kept the old Propeller GCC toolchain isolated from system
packages while still using Parallax's compiler and loader. The extracted compiler
reported `propeller-elf-gcc (propellergcc_v1_0_0_2411) 4.6.1`, consistent with
Parallax's Propeller C / PropGCC toolchain docs
([Propeller C reference](https://learn.parallax.com/support/C/propeller-c-reference),
[SimpleIDE Linux package](https://www.parallax.com/package/simpleide-software-for-linux-propeller-c/)).

The working WSL2 USB path was:

```text
g240 Windows USB port
  -> usbipd-win BUSID 1-6
  -> WSL2 vhci_hcd USB/IP device
  -> FTDI /dev/ttyUSB0
  -> propeller-load -b activityboard -p /dev/ttyUSB0
```

Windows identified the ActivityBot adapter as `USB Serial Converter`,
VID:PID `0403:6015`, BUSID `1-6`. After `usbipd bind --busid 1-6` and
`usbipd attach --wsl --busid 1-6`, WSL reported an FTDI serial device at
`/dev/ttyUSB0`. This matches Microsoft's supported WSL USB/IP workflow:
install `usbipd-win`, list devices, bind a USB device, then attach it to WSL
([Microsoft WSL USB docs](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)).

The first firmware proof used `scripts/parallax-run-smoke.sh`, which loaded
`firmware/smoke/hello.c` to RAM with `propeller-load -r -t` and intentionally
did not pass `-e`, so it did not write EEPROM. The loader detected
`Propeller Version 1`, sent the ELF, verified RAM OK, entered terminal mode, and
printed `sim2real Parallax smoke test`. Parallax's SimpleIDE guide distinguishes
RAM load/run from EEPROM load/run, so RAM-only is the right first bring-up step
([SimpleIDE User Guide](https://learn.parallax.com/sites/default/files/content/propeller-c-reference/landing/SimpleIDE-User-Guide-1-01.pdf)).

The second firmware proof used `firmware/smoke/encoder_monitor.c`, which reads
the original ActivityBot encoder pins directly: P14 for the left encoder signal
and P15 for the right encoder signal. Parallax's bundled `abdrive.h` documents
those same default encoder pins and P12/P13 as the default servo pins. The
bundled Parallax example `Test Encoder Connections.c` also uses direct reads of
P14/P15 and maps them to LEDs P26/P27 for a hand-spin wiring test; this matches
the legacy ActivityBot test-encoder-connections tutorial referenced in the
example source
([ActivityBot tutorials](https://learn.parallax.com/tutorials-menu/activitybot/),
[Propeller C ActivityBot legacy tutorial](https://learn.parallax.com/tutorials/robot/activitybot/legacy-version-propeller-c-programming-activitybot)).

The encoder monitor was loaded to RAM with:

```bash
timeout 45s sg dialout -c 'scripts/parallax-run-encoder-monitor.sh /dev/ttyUSB0'
```

It streamed `left_raw right_raw left_edges right_edges` over serial. While the
wheels were spun by hand, observed counts changed in real time, reaching
`left_edges=13` and `right_edges=46` during the test. This confirms that WSL,
the Propeller loader, the ActivityBot FTDI serial path, and the P14/P15 encoder
signals are usable from the SSH-driven workflow.

The WSL user `claude` was added to the `dialout` group so future sessions should
not need `sudo` to open `/dev/ttyUSB0`. Existing shells can use
`sg dialout -c '<command>'` until they are restarted.
