# Propeller inference speed report

Date: 2026-05-28

Scope: investigation of the current ActivityBot line-follow firmware,
Propeller 1 architecture, and practical ways to reduce policy decision latency.
This report has been updated to match the current three-sensor implementation
and measured H8/L0 timing.

## Executive summary

The best first speedup is not multiple floating-point worker cogs. The current
deployed `HIDDEN_SIZE=8`, `NUM_LAYERS=0` firmware is two bias-free linear
layers with no activation between them. That means the deployed policy is
exactly foldable into one `2 x 3` action matrix:

```text
action = decoder_action_rows * encoder * obs
```

For the current live firmware, that reduces action inference from 48 float
multiply/add terms to 6 terms, removes the unused value head and log-std from
deployment, and can be made bit-for-bit close in host tests before touching the
robot.

The second high-value speedup is to stop doing sequential QTI reads with a
1 ms charge delay per sensor. The current code charges and reads three active
sensors one at a time, so every loop pays at least 3 ms before any RC decay
time. A custom three-pin QTI read can charge all active sensors together and
measure them together, reducing sensor latency to about
`1 ms + max(sensor_decay)` instead of `3 ms + sum(sensor_decay)`. A timeout cap
is also important because the current `rc_time` default timeout is much larger
than the useful line-follow range.

The third immediate loop win was to remove the fixed
`pause(LINE_FOLLOW_LOOP_MS)` from the default drive decision path. The current
default is `firmware_loop_ms=0`; keep it that way unless deliberately testing a
deadline-based fixed-period controller.

After those, try `-mlmm` and `-O2/-mfcache` builds if they fit in hub RAM. The
current CMM drive ELF is still well under 32 KB by `propeller-elf-size`, so
there is room to test faster memory models without changing hardware.

## Current firmware path

Primary robot firmware:

- `ocean/line_follow/firmware/line_follow_model_live.c`
- `ocean/line_follow/scripts/parallax-build-line-follow-model-live.sh`
- generated weights at
  `ocean/line_follow/build/line-follow/line_follow_model_weights.h`

The build script compiles the firmware with:

```text
-Os -mcmm -m32bit-doubles -fno-exceptions -std=c99
```

and links the CMM simpletools/simpletext/simplei2c libraries. Drive-enabled
builds also link CMM `abdrive` and `fdserial`.

Measured local sizes from the existing built ELFs:

```text
line_follow_model_live.elf        text=9640   data=1164  bss=324   dec=11128
line_follow_model_live_drive.elf  text=15140  data=0     bss=3092  dec=18232
```

The currently generated live-model header contains:

```text
LINE_FOLLOW_MODEL_RAW_FLOATS    50
LINE_FOLLOW_MODEL_PADDED_FLOATS 57
LINE_FOLLOW_MODEL_BYTES         200
```

For `HIDDEN_SIZE=8`, `NUM_LAYERS=0`, those 50 raw floats are:

```text
encoder weights: H * obs = 8 * 3 = 24
decoder weights: (2 actions + 1 value) * H = 3 * 8 = 24
continuous log_std: 2
total = 50
```

The robot only uses `decoder_out[0]` and `decoder_out[1]` as motor actions.
The value output and `log_std` are training/runtime artifacts that are not used
for deterministic robot control.

## Current loop anatomy

The live loop in `line_follow_model_live.c` does this every iteration:

1. Read three QTI sensors.
2. Normalize raw QTI readings into `obs_q1000`.
3. Convert those integers to float observations.
4. Run `forward_model`.
5. Convert action floats to ActivityBot ticks per second.
6. Optionally print a telemetry row.
7. Optionally call `drive_speed`.
8. Update debug LEDs.
9. Sleep with `pause(LINE_FOLLOW_LOOP_MS)`.

Important details from the code:

- `read_qti_pin` does `high(pin); pause(1); rc_time(pin, 1);`.
- `read_qti` calls `read_qti_pin` sequentially for each of the three sensors.
- `forward_model` runs encoder linear, then decoder linear when
  `NUM_LAYERS == 0`.
- `DECODER_SIZE` is 3, but only action outputs 0 and 1 are used.
- `action_to_ticks` clamps negative actions to zero before converting to ticks.
- The loop calls `pause(LINE_FOLLOW_LOOP_MS)` at the end only when the configured
  pause is greater than zero.

The ActivityBot drive library also matters:

- `drive_speed` starts the `abdrive` encoder/servo process in another cog via
  `cogstart(encoders, ...)` when needed.
- `abdrive` has a 20 ms servo/control cadence internally.
- `drive_speed` can insert a 120 ms pause on sign crossings. The current model
  firmware clamps negative wheel commands to zero, which should reduce this
  risk, but it is still worth avoiding unnecessary stop/reverse transitions.

## Current timing evidence

Existing local telemetry logs show the model-size effect clearly:

```text
H16/L1 old recurrent path:
  ocean/line_follow/build/line-follow/model-live-loop-ms-print50-150.txt
  host_loop_ms p50 = 476.6 ms

H8/L1 recurrent path:
  ocean/line_follow/build/line-follow/model-live-hidden8-timing.txt
  host_loop_ms p50 = 187.5 ms

H8/L0 feed-forward path, telemetry-only:
  ocean/line_follow/build/line-follow/model-live-ff8-timing.txt
  host_loop_ms p50 = 52.3 ms

H8/L0 feed-forward path, drive log:
  ocean/line_follow/build/line-follow/model-live-ff8-drive.txt
  host_loop_ms p50 = 64.6 ms
```

Those H8/L0 measurements used `loop_ms=25` and `print_every=50`, so the measured
period includes a fixed 25 ms sleep plus occasional serial output. The useful
work is therefore materially less than 52 ms, but still too slow to leave as-is
if the robot is overrunning the line.

## Propeller architecture constraints

The Propeller P8X32A has eight independent 32-bit cogs. Each cog has its own
512-long cog RAM, while the 32 KB main RAM is shared through a round-robin hub.
Parallax's Propeller 1 page lists 20 MIPS per cog, 160 MIPS total, 32 KB global
RAM, and 512 x 32-bit cog RAM per core:

- https://www.parallax.com/propeller-1/

Parallax's P8X32A Q&A explains the same core model: cogs run independently, each
has its own memory and counter hardware, and cogs share main memory via the hub:

- https://www.parallax.com/propeller/qna/Content/QnaTopics/QnaP8X32AIntro.htm

The practical consequence: native PASM running from cog RAM is fast and
deterministic. CMM/LMM C code is larger-scale and convenient, but hot C loops
fetch through hub memory. More C cogs do not automatically make one small
numeric calculation faster, because they still communicate through hub memory
and still use software arithmetic.

Parallax's Propeller C multicore docs show that C functions can be launched in
other cogs with `cog_run` or `cogstart`, but they need stack space and shared
global state:

- https://learn.parallax.com/courses/multicore-approaches/lessons/simple-multicore/
- https://learn.parallax.com/courses/propeller-c-functions/lessons/multicore-example/

Parallax's C library study notes that LMM executes faster than CMM but takes more
memory:

- https://learn.parallax.com/courses/propeller-c-library-studies/lessons/create-and-test-the-library/

The local compiler also confirms the available target modes:

```text
-mcmm   compressed memory model, code and data in hub
-mlmm   large memory model, code and data in hub
-mcog   code runs in internal cog space
-mfcache load external code into cog memory fcache
```

## Finding 1: fold the H8/L0 model exactly

Current deployed `NUM_LAYERS=0` inference is:

```c
linear_forward(obs, encoder_w, encoder_out, OBS_SIZE, HIDDEN_SIZE);
linear_forward(encoder_out, decoder_w, decoder_out, HIDDEN_SIZE, DECODER_SIZE);
actions[0] = decoder_out[0];
actions[1] = decoder_out[1];
```

There is no activation between the two linear layers. Therefore:

```text
decoder * (encoder * obs) == (decoder * encoder) * obs
```

For the current H8/L0 deployment:

```text
current action math:
  encoder: 8 outputs * 3 inputs = 24 multiply/add terms
  decoder: 3 outputs * 8 hidden = 24 multiply/add terms
  total: 48 terms, plus temporary arrays and float movement

folded action-only math:
  2 actions * 3 inputs = 6 multiply/add terms
```

If the value output is kept for debugging, the folded matrix is `3 x 3`, which
is still only 9 terms. If only left/right actions are deployed, the weights
drop from 50 raw floats to 6 action coefficients. At 32-bit float, that is
200 bytes down to 24 bytes. At Q15 fixed point, that is 12 bytes.

This is exact for the currently deployed architecture. It does not require
retraining. It only requires an offline converter step and a host equivalence
test against the existing two-layer code.

Recommended implementation direction:

1. Add an offline conversion mode that loads the native checkpoint and emits:

   ```text
   folded_action_w[2][3] = decoder_action_w[2][H] * encoder_w[H][3]
   ```

2. Keep a host test that compares old and folded actions on:

   - the existing canned smoke observations,
   - observations parsed from real QTI logs,
   - random observations in `[0, 1]`.

3. Deploy the folded float version first.
4. Then convert the folded weights to fixed point.

## Finding 2: use fixed point, not half-float

The current firmware already normalizes QTI readings as integers:

```text
obs_q1000 = clamp((raw - white) * 1000 / (black - white), 0, 1000)
```

It then converts those values to float just to feed the model. That is avoidable.

Recommended fixed-point deployment:

```text
obs_q15 = obs_q1000 * 32767 / 1000
weight_q15 = round(folded_weight * scale)
accum_q30 = sum(obs_q15 * weight_q15)
action_q15 = accum_q30 >> 15
```

Then clamp and convert to ticks with integer math. The current max speed is
small:

```text
1.0 action -> 0.038 m/s
0.038 m/s with 65 mm tires and 64 ticks/rev -> about 12 ticks/s
```

The current deployment mapping is non-reversing: `action=-1` maps to stopped,
`action=0` maps to about 6 ticks/s, `action=0.5` maps to about 9 ticks/s, and
`action=1` maps to about 12 ticks/s. So deployment can compute an action score
in fixed point, apply the 0.04 deadband threshold in the same scale, clamp
negatives to zero, and map to ticks.

Why not fp16 or smaller floats?

- The Propeller 1 does not have hardware floating-point support in this path.
- Storing weights as fp16 only saves memory unless the firmware can compute in
  fp16 directly.
- Converting fp16 to fp32 each loop can make runtime slower.
- Custom fp16 arithmetic is more work than Q15/Q12 fixed point and less aligned
  with the existing integer QTI normalization.

General TinyML practice also points toward integer quantization for small
microcontrollers. LiteRT's 8-bit quantization spec represents real values with
integer values, zero-points, and scales, and defines fully-connected int8
weights with int32 bias/accumulation:

- https://ai.google.dev/edge/litert/conversion/tensorflow/quantization/quantization_spec

CMSIS-NN is ARM-specific and cannot be used on Propeller, but it is a useful
reference point: its purpose is efficient neural-network kernels for
microcontrollers, and it follows int8/int16 TensorFlow Lite Micro quantization:

- https://arm-software.github.io/CMSIS_6/main/NN/index.html

For this robot, do not import a framework. The model is tiny enough to write the
fixed-point math directly.

## Finding 3: QTI read latency is a major target

Parallax describes the QTI sensor as Charge Transfer Infrared. Lower RC-time
values mean a lighter or closer reflective target; higher values mean darker,
less reflective, or farther:

- https://www.parallax.com/product/qti-sensor/
- https://learn.parallax.com/kickstarts/qti-sensor/

The current firmware reads the three active sensors sequentially:

```text
for each pin:
  high(pin)
  pause(1)        # 1 ms charge delay
  rc_time(pin, 1)
```

Minimum charge delay alone is 3 ms. The RC-time terms are then added serially.
Using observed real readings:

```text
normal white/edge values: roughly 30 to 350 us
off-paper or timeout-like values: can be much higher
```

Current rough latency:

```text
sequential typical: 3 ms charge + sum(decays) ~= 3.1 to 4.5 ms
sequential high/raw: 3 ms charge + multiple high decays, possibly much higher
```

A better QTI path:

```text
set all three active QTI pins high together
wait one charge interval
set all three active pins to input together
loop until each bit falls or timeout
return three elapsed times
```

Expected rough latency:

```text
parallel typical: 1 ms charge + max(decay) ~= 1.1 to 1.4 ms
parallel capped: 1 ms charge + configured timeout cap
```

That should save several milliseconds every loop and, more importantly, removes
long stalls when one sensor sees a very dark or off-paper condition.

Implementation options:

- Best: one PASM cog continuously samples all active QTI pins and publishes latest
  raw values plus a sample counter to hub memory.
- Simpler C prototype: charge all active pins, then poll `INA` for a bit mask
  until all have decayed or a deadline expires.
- If retaining `rc_time`, use helper cogs to split the sensors, because each
  cog has two counter modules. This is less clean than one custom grouped-pin
  sampler but may be easier to prototype.

Also set an explicit line-follow timeout. Local `simpletools` docs say
`rc_time` uses 1 us units by default and the timeout defaults to 1/4 second.
The model already clamps values above `QTI_BLACK_TIME=350` to black, so waiting
far beyond that does not add useful policy information.

Recommended first timeout experiments:

```text
QTI timeout cap: 800 us, 1200 us, 2000 us
charge delay: 1000 us first, then test 230 us and 500 us
```

Parallax's BlocklyProp QTI line-follow tutorial uses a fast digital timing cycle:
charge/output for about 230 us, input for about 230 us, then read black/white
states. It documents black as 1 and white as 0:

- https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/

That opens an even faster path: train a binary-QTI policy or add a binary
fallback controller. It loses analog edge detail, so I would try parallel analog
QTI first.

## Finding 4: remove fixed sleep from the decision path

The firmware has:

```c
pause(LINE_FOLLOW_LOOP_MS);
```

at the end of every loop. Older builds used `firmware_loop_ms=25`, which meant
every loop waited 25 ms after the work was already finished. Current line-follow
firmware defaults to `firmware_loop_ms=0`; the measured H8/L0 three-sensor loop
is about 24 ms with sparse telemetry.

Better control-loop behavior:

```text
loop_start = CNT
read sensors
run inference
send drive command
if fixed-period mode:
  wait until loop_start + target_period only if still early
else:
  immediately begin next sensor sample
```

For drive tests, run no fixed post-work sleep unless the `abdrive` layer behaves
badly when commands arrive faster than its 20 ms servo cadence. Sending commands
faster than 20 ms should simply update the requested speed before the next
internal servo/control cycle.

Expected gain: up to 25 ms from the current H8/L0 build before any math changes.

## Finding 5: reduce or remove telemetry in drive mode

Serial telemetry is useful for calibration but is poison for tight control.
At 115200 baud, a long line can cost milliseconds on the wire. More importantly,
printing from the control loop adds jitter exactly when the robot needs
consistent reaction time.

Current code prints telemetry when:

```text
LINE_FOLLOW_PRINT_EVERY <= 1 || loops % LINE_FOLLOW_PRINT_EVERY == 0
```

That means `LINE_FOLLOW_PRINT_EVERY=0` would currently print every loop, not
disable printing.

Recommended deployment behavior:

- Add a true `LINE_FOLLOW_PRINT_EVERY=0` means no per-loop telemetry.
- For drive tests, print only a startup banner and maybe a compact final summary.
- If live visibility is required, publish one byte or a small binary packet at a
  low rate from a separate diagnostic mode, not from the main control loop.

The current `print_float3` function avoids `%f`, which is good. The bigger issue
is that any serial row inside the control loop can stall or jitter the decision
period.

## Finding 6: try faster C build modes after the algorithmic fixes

Current builds use CMM because it is compact. Parallax's docs say LMM is faster
but larger. The current drive-enabled ELF is about 18 KB of text/data/bss by
`propeller-elf-size`, so there may be enough room to test LMM.

Recommended build experiments after a no-code instrumentation pass:

```text
Baseline current:
  -Os -mcmm -m32bit-doubles

Try:
  -Os -mlmm -m32bit-doubles       with lmm simple libraries
  -O2 -mlmm -m32bit-doubles       if size fits
  -O2 -mcmm -mfcache              if CMM size must stay small
```

Do not assume LMM is safe until it is loaded on the robot and timed. The build
must link the matching `lmm/` simple library archives, not the current `cmm/`
archives.

Expected gain: medium. It can speed C code, but it will not remove the biggest
structural costs by itself.

## Finding 7: a PASM cog helps, but use it surgically

The Propeller is strongest when a cog owns one timing-critical task. Good uses
for extra cogs in this robot:

- One QTI sampler cog that continuously publishes fresh sensor readings.
- One tiny fixed-point inference cog if the main C loop remains too slow.
- The existing `abdrive` cog for servo pulses and encoder feedback.

Bad first use:

- Four C cogs splitting fp32 matrix multiplies.

Why multiple fp32 C worker cogs are likely low return:

- The current H8/L0 model can be folded to 6 action MACs, so there is almost
  nothing to parallelize.
- CMM/LMM worker cogs need stacks in hub RAM and communicate through volatile
  hub variables.
- Multiple CMM/LMM cogs all fetch code/data through the shared hub.
- Software float remains software float.
- Synchronization overhead can exceed the saved arithmetic.

If native math is still needed, use a single PASM or COGC kernel that keeps the
small folded weights in cog memory and computes the whole action in one place.
For fixed-point H8/L0 folded inference, that kernel can be extremely small.

Parallax's floating-point routines are still useful as a reference. Their
floating-point PDF shows that assembler-level float operations are much faster
than high-level calls, and the basic float support can fit in cog memory:

- https://forums.parallax.com/uploads/attachments/53599/69870.pdf

But for this model, fixed point is simpler and should be faster than building a
floating-point coprocessor path.

## Finding 8: digital or hybrid controller fallback may beat any neural path

The existing `line_follow_sanity.c` policy stub is integer-only and already
matches the basic three-QTI control idea:

```text
left_dark  = left
right_dark = right
correction = left_dark - right_dark
```

That style of controller can run at QTI sampling speed and can serve as:

- a baseline to prove the physical robot can follow at a given speed,
- a safety fallback when the model is uncertain or sensors saturate,
- a teacher policy for retraining a smaller learned policy.

If the current learned policy still overruns the line after latency is reduced,
the next training pass should include the actual measured control delay and
possibly binary/digital QTI observations. Do not add recurrence unless the
Propeller inference path is redesigned, because H8/L1 was measured at about
187 ms and H16/L1 at about 477 ms in existing logs.

## Recommended experiment order

### Phase 1: measure before changing behavior

Add temporary timing buckets around:

```text
read_qti
forward_model
action_to_ticks
drive_speed
telemetry print
pause
whole loop
```

Use `CNT` and/or a GPIO pulse. Parallax AN009 documents using the system counter
and pin toggling for execution timing:

- https://www.parallax.com/package/an009-code-execution-time-on-the-p8x32a-2/

Keep this as a temporary instrumentation build. Do not drive from an
instrumented serial-heavy loop.

### Phase 2: remove avoidable loop delay

Test H8/L0 with:

```text
LINE_FOLLOW_LOOP_MS=0
LINE_FOLLOW_PRINT_EVERY=0  # after code supports true off
```

If the code has not yet been changed to support print-off, use a very large
print interval for the timing test.

Success target: get the current float H8/L0 loop under 30 ms before changing
math.

### Phase 3: exact folded float model

Emit folded action weights and replace the two-layer action computation with one
small matrix multiply.

Verification:

```text
host: old two-layer actions vs folded actions
firmware fake obs: old ticks vs folded ticks
live telemetry on stand: same action signs and clamps
```

Success target: action difference small enough that tick outputs match across
real QTI logs.

### Phase 4: folded fixed-point model

Use `obs_q1000` directly. Convert folded weights to Q15 or Q12. Accumulate in
32-bit signed integers.

Verification:

```text
host: float folded vs fixed folded over canned/random/log observations
robot fake obs: tick outputs match or differ only at threshold boundaries
```

Success target: model math no longer appears as a meaningful timing bucket.

### Phase 5: parallel/capped QTI

Prototype grouped three-pin QTI read. First in C, then PASM if C polling is too
slow or jittery.

Verification:

```text
static surface: raw distributions match current rc_time enough for normalization
line sweep: sensor order and polarity unchanged
off-paper: timeout cap prevents long loop stalls
```

Success target: QTI bucket around 1 to 3 ms for normal reads, capped for lost
line.

### Phase 6: LMM/O2/mfcache build tests

Only after the above, test C compiler/memory mode changes. Keep exact same
behavior and compare timing buckets.

Success target: measurable improvement with ELF still fitting and robot behavior
unchanged.

### Phase 7: optional dedicated cogs

If the loop is still too slow:

- move QTI sampling to one dedicated cog,
- optionally move fixed-point inference to one native cog,
- leave main C loop as coordinator and `drive_speed` caller.

Avoid a four-cog fp32 design unless measurements show folded fixed-point and QTI
work are not enough.

## Expected impact table

| Change | Expected impact | Risk | Notes |
| --- | --- | --- | --- |
| Keep fixed post-loop sleep disabled | Already applied | Low | Current default is `firmware_loop_ms=0`; reintroduce only as a deadline wait. |
| Disable drive telemetry | Medium | Low | Reduces jitter and serial stalls. |
| Fold H8/L0 model | High | Low | Exact for current architecture. 48 MACs down to 6 action MACs. |
| Remove value/log_std from deploy | Low to medium | Low | Mostly memory/code cleanliness unless combined with folding. |
| Fixed-point folded inference | High | Medium | Eliminates software float from model path. Needs equivalence tests. |
| Grouped/parallel QTI read | High | Medium | Saves several ms and prevents long dark/off-paper stalls. |
| Explicit QTI timeout cap | High for failure cases | Low | Prevents catastrophic `rc_time` waits. |
| LMM instead of CMM | Medium | Medium | Faster but larger; must link LMM libraries and verify fit. |
| `-O2/-mfcache` | Medium | Medium | May speed hot loops, may grow code. |
| One PASM QTI sampler cog | High | Medium/high | Best sensor architecture if C prototype is too slow. |
| One PASM fixed-point inference cog | Medium/high | Medium/high | Useful if main C fixed-point is still too slow. |
| Four C cogs for fp32 math | Low or negative | High | Too much hub/sync/soft-float overhead for current tiny model. |
| fp16 weights with fp32 compute | Low | Medium | Saves storage, likely not runtime. |

## Concrete answer on "can we use 4 cores for floating point?"

Technically yes, but it is not the right first design for this robot.

The current model has only 8 useful folded dot-product terms. Four fp32 worker
cogs would add:

- worker stacks,
- hub-memory mailboxes,
- synchronization,
- CMM/LMM hub fetch pressure,
- software-float calls in each worker,
- result collection latency.

That can easily be slower than one cog doing a tiny fixed-point calculation.

The Propeller multicore win is better spent on:

```text
Cog 0: main coordination and drive_speed calls
Cog 1: abdrive encoder/servo process
Cog 2: QTI sampler, preferably PASM
Cog 3: optional fixed-point inference kernel, only if needed
```

This layout keeps timing-sensitive I/O and math isolated without trying to
parallelize a computation that can be reduced to almost nothing.

## Sources

Local code and logs:

- `ocean/line_follow/firmware/line_follow_model_live.c`
- `ocean/line_follow/scripts/parallax-build-line-follow-model-live.sh`
- `ocean/line_follow/scripts/generate-line-follow-model-header.py`
- `ocean/line_follow/build/line-follow/line_follow_model_weights.h`
- `ocean/line_follow/scripts/analyze-line-follow-live-log.py`
- `ocean/line_follow/build/line-follow/model-live-loop-ms-print50-150.txt`
- `ocean/line_follow/build/line-follow/model-live-hidden8-timing.txt`
- `ocean/line_follow/build/line-follow/model-live-ff8-timing.txt`
- `ocean/line_follow/build/line-follow/model-live-ff8-drive.txt`
- `tools/parallax/simpleide/opt/parallax/Workspace/Learn/Simple Libraries/Utility/libsimpletools/simpletools.h`
- `tools/parallax/simpleide/opt/parallax/Workspace/Learn/Simple Libraries/Utility/libsimpletools/source/rcTime.c`
- `tools/parallax/simpleide/opt/parallax/Workspace/Learn/Simple Libraries/Robotics/ActivityBot/libabdrive/abdrive.c`
- `tools/parallax/simpleide/opt/parallax/Workspace/Learn/Simple Libraries/Robotics/ActivityBot/libabdrive/abdrive.h`

Online references:

- Parallax Propeller 1 architecture and specifications:
  https://www.parallax.com/propeller-1/
- Parallax P8X32A Q&A:
  https://www.parallax.com/propeller/qna/Content/QnaTopics/QnaP8X32AIntro.htm
- Parallax Propeller C simple multicore:
  https://learn.parallax.com/courses/multicore-approaches/lessons/simple-multicore/
- Parallax Propeller C `cogstart` multicore example:
  https://learn.parallax.com/courses/propeller-c-functions/lessons/multicore-example/
- Parallax Propeller C CMM/LMM library study:
  https://learn.parallax.com/courses/propeller-c-library-studies/lessons/create-and-test-the-library/
- Parallax AN009 code execution timing:
  https://www.parallax.com/package/an009-code-execution-time-on-the-p8x32a-2/
- Propeller floating-point routines PDF:
  https://forums.parallax.com/uploads/attachments/53599/69870.pdf
- Parallax QTI sensor product page:
  https://www.parallax.com/product/qti-sensor/
- Parallax QTI KickStart:
  https://learn.parallax.com/kickstarts/qti-sensor/
- Parallax ActivityBot QTI digital timing lesson:
  https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/
- LiteRT 8-bit quantization specification:
  https://ai.google.dev/edge/litert/conversion/tensorflow/quantization/quantization_spec
- CMSIS-NN overview:
  https://arm-software.github.io/CMSIS_6/main/NN/index.html
