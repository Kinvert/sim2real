# PufferLib Line-Following Sim2Real Spec

This project is the line-following sim2real path for a Parallax ActivityBot with
a QTI Line Follower kit. The target real task is simple: drive on white paper and
follow black marker or tape lines using the four QTI sensors.

The project lives in:

```text
ocean/line_follow/
```

The env itself must follow normal PufferLib Ocean conventions: C env code,
simple readable files, high steps per second, local raylib render support, and no
large external physics dependency.

## Keith Notes

I will add notes as I find them. Once you write them up better and put them in the right place you can get rid of the comment I made. Don't delete this section, just update what is needed in other md's etc, and then you can delete that single bullet point here in Keith Notes.

## Goal

Train a small PufferLib policy in simulation, export it into a tiny format, and
run it on the Parallax ActivityBot using Propeller C.

The policy deployed to the robot should consume the same four normalized QTI
readings and choose the same wheel-speed actions used in sim. The robot-side code
should not depend on Python, Torch, ONNX Runtime, or hosted OS libraries.

## Current Proven Hardware Workflow

The g240 workflow is proven:

```text
5090 user prompt
  -> Codex in g240 WSL
  -> build Propeller C firmware
  -> propeller-load sends ELF to ActivityBot RAM over /dev/ttyUSB0
  -> ActivityBot runs the program
  -> serial telemetry returns to WSL
  -> raylib dashboard renders on g240 DISPLAY=:0
```

The working bring-up code has been moved under this env:

```text
ocean/line_follow/firmware/
ocean/line_follow/host/
ocean/line_follow/scripts/
ocean/line_follow/docs/
```

The existing encoder dashboard is not the final line-follower policy runner. It
is a hardware proof that Codex can build, load, run, and observe ActivityBot
programs from this repo.

## PufferLib Ethos For This Env

- Keep the env in C.
- Keep the inner step loop simple and fast.
- Prefer analytic math over pixel maps where possible.
- Avoid a full physics engine for v1.
- Make the env easy to inspect with raylib.
- Randomize the parts that will not match real life exactly.
- Keep the policy input/output compatible with Propeller deployment.
- Make the code "look like" other PufferLib envs like Breakout, 2048, Sokoban, etc.
- When in doubt, check `ocean/breakout`, `ocean/g2048`, `ocean/boxoban`,
  and `ocean/whisker_racer` and copy their structure before inventing a new
  pattern. These are the templates for this env.

## V1 Scope

V1 should be the smallest trainable environment that can plausibly transfer:

- A differential-drive robot.
- Four QTI-like downward-facing sensors with analog-like RC decay readings.
- A white floor with a black line represented analytically.
- Two continuous actions that map directly to left/right wheel speeds.
- Domain randomization for geometry, sensing, and drive response.
- A raylib render mode for visual debugging when not running headless.
- A simple reward based on line progress and staying recoverable.

V1 does not need:

- Photorealistic sensor rendering.
- Full wheel-ground contact physics.
- Camera input.
- Obstacles.
- Intersections, forks, or maze solving.
- Learned recurrent memory unless the feed-forward policy is clearly inadequate.

Training should otherwise use the normal PufferLib stack. PPO details, MinGRU
versus MLP, and related policy choices are training choices, not env semantics.
The hard deployment contract is:

```text
explicit current obs[4] -> policy_forward in simple C -> continuous action[2]
```

Do not add hidden env observations such as previous QTI readings unless they are
explicitly part of the observation and can be reproduced on the robot. If a
recurrent network is ever used, its recurrent state must be an explicit exported
model state that the Propeller C runner updates exactly; it is not a secret extra
environment input.

## Stretch Goals

Once the normal four-sensor policy works, add sensor-failure robustness. The env
should be able to randomize an episode where zero, one, or two QTI sensors are
failed. A failed sensor should look like a plausible real failure mode, such as
stuck-white, stuck-black, stuck-at-calibrated-midpoint, high noise, or unplugged
timeout. The purpose is to make the deployed robot degrade gracefully if a QTI
wire loosens or a sensor is unplugged mid-run.

This is not a v1 requirement. It should be introduced after the base task trains,
because it can make early learning harder.

## Real Robot Assumptions

The real robot is a Parallax ActivityBot differential-drive robot with:

- Four QTI line sensors mounted under the front of the chassis.
- A white floor/paper background.
- A black marker/tape line.
- Wheel speed commands implemented through Parallax drive libraries.
- Propeller C as the deployment language.

Parallax documents the QTI as a Charge Transfer Infrared sensor: the
microcontroller charges a capacitor and measures how long it takes to decay
through an infrared phototransistor. The decay time changes with reflected IR,
so it is naturally a scalar reflectance reading, even though Parallax's
BlocklyProp line-following tutorial also shows a fixed-timing digital read where
black returns `1` and white returns `0`
([QTI product page](https://www.parallax.com/product/qti-sensor/),
[QTI KickStart](https://learn.parallax.com/kickstarts/qti-sensor/),
[QTI sensor test tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/)).

For this env, the policy observation should be the raw-ish scalar signal, not a
hardcoded binary mask. We can still compute thresholded bits for debugging,
heuristics, and a simple baseline.

Downloaded reference copies live locally under this ignored path:

```text
ocean/line_follow/docs/vendor/parallax-qti/
```

Observation order must be stable in sim and real:

```text
[outer_left, inner_left, inner_right, outer_right]
```

If the real wiring ends up in a different physical order, robot-side code should
reorder the pins before passing observations into the policy. The policy should
not need to know board pin numbers.

## Units

The sim should use SI units internally:

```text
position: meters
angle: radians
time: seconds
wheel speed: meters/second internally
robot command: continuous [-1, 1] left/right action -> left/right speed pair
```

Deployment code may use ActivityBot drive ticks/second. Conversion from policy
action to real `drive_speed(left, right)` ticks/second belongs in the robot-side
adapter.

## State

Each env instance needs at least:

```text
robot_x, robot_y, robot_theta
left_wheel_velocity, right_wheel_velocity
previous_action
track_id / track parameters
nearest_track_progress
lost_line_steps
episode_step
randomized sensor geometry
randomized drive parameters
randomized sensor parameters
track primitives / sampled centerline arrays
optional nearest segment cache per sensor
optional pre-generated track cache
```

The policy observation should stay small. Extra privileged state such as nearest
track progress can be used for reward, logging, and debug rendering, but not for
policy input.

## Track Model

Track generation should not copy Whisker Racer's track generator. Whisker Racer
is useful to peek at for how a vehicle env stores track samples and renders a
course, but its Bezier/control-point generator should not be treated as the
template for line following.

The line-following track should be represented as a geometric centerline plus a
black-line width. The black line is the swept strip around that centerline, with
edge softness and reflectivity variation. The line width should be randomized
around real marker/tape widths; start around 5 mm for marker-like lines, then
widen the range after the basic policy trains.

The track representation should support two related forms:

```text
primitive path -> sampled centerline polyline -> fixed-size track arrays
```

The primitive path is the source of truth for generation. The sampled polyline is
the fast runtime representation used by reset, reward progress, sensor distance,
and rendering. Sampling into fixed arrays keeps the hot loop simple and avoids
dynamic allocation during stepping.

Track generation speed is a first-class requirement. A death/reset may generate
a new track, and resets directly affect training SPS. The generator must produce
valid curriculum-appropriate tracks with bounded work and low constant factors.
Correctness matters, but not through expensive unbounded search.

Start with simple generated tracks:

- Straight segment.
- Gentle arc sequences.
- Oval or rounded-rectangle loop.
- S-curve built from alternating arcs.
- Piecewise straight + circular-arc route with bounded curvature.

Do not use bitmap image sampling for v1. Geometric distance-to-line from fixed
track primitives or sampled polyline segments is faster, deterministic, and
easier to randomize.

### Track Primitive Generator

Use a small "turtle" path generator for v1 instead of random Bezier control
points:

```text
pose = start x/y/heading
append LINE(length)
append ARC(radius, signed_angle)
append LINE(length)
...
```

Each primitive appends centerline samples and cumulative progress. A line has
constant heading. An arc has constant radius and smooth heading change. Adjacent
line and arc primitives are tangent-continuous when constructed from the current
pose, so smooth radiused turns are easy to produce and easy for the robot to
follow.

The generator should reject tracks that are out of bounds, self-intersect too
closely, have turns below the configured minimum radius, or leave too little
space between separate parts of the line. Rejection sampling is acceptable at
reset as long as it has a retry cap and falls back to a known-good template.

Generation should be fast enough that reset-time track creation is not visible
in SPS for v1. Prefer algorithms that are correct by construction:

- For straights, arcs, ovals, rounded rectangles, and S-curves, use deterministic
  template families with randomized dimensions instead of open-ended random
  search.
- Validate with cheap bounds checks and local segment-spacing checks, not full
  expensive geometry unless the curriculum requires it.
- Keep `MAX_TRACK_PRIMITIVES` and `MAX_TRACK_SAMPLES` fixed. If a sampled track
  would exceed the cap, regenerate or clamp at reset, never allocate.
- Use a capped retry loop, for example `MAX_TRACK_GEN_ATTEMPTS`, then fall back
  to a known-good simple track for that curriculum level.
- Precompute per-sample tangent, normal, cumulative progress, line width, and
  reflectance during reset so step-time sensor reads are cheap.

If later sharp-corner or maze-like curricula make generation expensive, add a
per-worker ring buffer or cache of already generated tracks. The training step
should never wait on a slow, unbounded generator when a valid cached track can be
used.

### Sharp And 90-Degree Turns

The long-run env should support both smooth radiused turns and sharp 90-degree
turns. Do not force every track through a smooth spline.

Add this as a curriculum after smooth tracks work:

- **Radiused 90s:** generate a Manhattan/grid route and replace each corner with
  a circular fillet of randomized radius. This gives realistic tape/marker
  layouts with smooth robot-friendly corners.
- **Sharp 90s:** represent the black line as the union of thick straight
  segments with square, mitered, or round joins. Sensor coverage near a corner
  should use distance to the union of adjacent segments, not only distance to a
  single nearest centerline point.
- **Mixed tracks:** randomly combine long straights, smooth arcs, and occasional
  sharp corners once the base policy is stable.

For v1, smooth primitives are enough. The data model should still leave room for
corner mode so adding sharp turns later does not require rewriting the env.

### Runtime Distance And Progress

For each sensor, find the nearest sampled centerline segment and compute signed
lateral distance plus nearest progress. This is geometric polyline sampling, not
bitmap sampling. A fixed-size sample array is acceptable for v1:

```text
TrackSample {
  x, y
  tangent_x, tangent_y
  normal_x, normal_y
  progress_s
  line_width_m
  black_reflectance
}
```

If this becomes a bottleneck, add a simple spatial grid or cache the previous
nearest segment index per env/sensor. Do not add that complexity until profiling
shows it matters.

Sensor line coverage should use a soft edge, not a hard binary line boundary:

```text
coverage = smoothstep(half_width + edge_softness,
                      half_width - edge_softness,
                      abs(signed_lateral_distance))
```

Line reflectivity should vary slowly along progress so the black marker is not
perfectly consistent:

```text
black_reflectance_at_s = base_black_reflectance + low_frequency_noise(s)
line_width_at_s        = base_line_width + low_frequency_width_noise(s)
```

Track generation should expose:

```text
line_width_m
line_width_variation_m
line_edge_softness_m
line_reflectance_noise
track_length_m
track_family
track_num_primitives
track_straight_length_min_m
track_straight_length_max_m
turn_radius_min_m
turn_radius_max_m
arc_angle_min_rad
arc_angle_max_rad
corner_mode
corner_fillet_radius_m
track_bounds_m
max_track_gen_attempts
max_track_primitives
max_track_samples
track_cache_size
```

The reward should use nearest-progress along the centerline, but the observation
must come only from the QTI sensor model.

## Robot Kinematics

Use a differential-drive kinematic model:

```text
v_left  = filtered commanded left wheel speed
v_right = filtered commanded right wheel speed
v       = (v_left + v_right) / 2
omega   = (v_right - v_left) / wheel_base

x     += v * cos(theta) * dt
y     += v * sin(theta) * dt
theta += omega * dt
```

V1 should include simple command lag:

```text
filtered_speed += alpha * (commanded_speed - filtered_speed)
```

The lag parameter should be randomized so policies do not depend on exact motor
response.

Suggested sim control rate:

```text
dt = 0.05 seconds
20 Hz policy/action rate
```

That is slow enough to match a microcontroller control loop and fast enough for
line-following dynamics.

## QTI Sensor Facts

Official Parallax sources imply these constraints:

- The QTI sensor uses an infrared LED plus infrared phototransistor for
  close-range reflective sensing and can distinguish dark/low-reflectivity
  surfaces from light/high-reflectivity surfaces
  ([QTI product page](https://www.parallax.com/product/qti-sensor/),
  [QTI KickStart](https://learn.parallax.com/kickstarts/qti-sensor/)).
- It can be wired/used either as a fast digital black/white sensor or as an
  analog-like sensor for shades of gray. The analog-like communication is
  capacitor decay time, not voltage sampled by an ADC
  ([QTI product page](https://www.parallax.com/product/qti-sensor/)).
- The KickStart examples report the current QTI value as a number; lower values
  mean lighter/more reflective or closer, higher values mean darker/less
  reflective or farther. Parallax gives typical ranges on the order of tens to
  thousands, depending on microcontroller and surface
  ([QTI KickStart](https://learn.parallax.com/kickstarts/qti-sensor/)).
- Propeller C already has `simpletools` support for RC decay measurement:
  charge the pin high, wait briefly, then call `rc_time(pin, 1)`. Parallax's
  Sense Light lesson says `rc_time` switches the pin to input and measures how
  long the voltage takes to decay past the Propeller input threshold
  ([Propeller C Sense Light](https://learn.parallax.com/courses/propeller-c-simple-circuits/lessons/sense-light/)).
- Parallax's ActivityBot Blockly QTI function uses a digital shortcut: set the
  QTI pins high/output, flash for about 230 us, switch to input for about 230 us,
  then read the pin states. In that mode, reflection seen is `0`, reflection not
  seen is `1`, so black line returns `1` and white surface returns `0`
  ([QTI sensor test tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/test-the-qti-sensors/)).
- The QTI Line Follower kit mounts sensors under the ActivityBot chassis; each
  sensor connects to 5 V, one Propeller I/O pin, and ground
  ([QTI build tutorial](https://learn.parallax.com/courses/qti-line-following-activitybot-with-blocklyprop/lessons/build-the-qti-line-follower/)).

## Sensor Model

There are four downward-facing QTI sensors rigidly attached to the robot. Sensor
world positions are computed from robot pose and randomized body-frame offsets.

Nominal body-frame layout:

```text
outer_left   x = -sensor_outer_x, y = sensor_forward_y
inner_left   x = -sensor_inner_x, y = sensor_forward_y
inner_right  x =  sensor_inner_x, y = sensor_forward_y
outer_right  x =  sensor_outer_x, y = sensor_forward_y
```

Each sensor samples local surface reflectance under a small footprint, not just
whether its center point is inside the black line. The simplest useful model is:

```text
line_coverage = footprint overlap with black line, approximately [0, 1]
surface_reflectance = mix(white_reflectance, black_reflectance, line_coverage)
decay_time = rc_offset + rc_gain / max(surface_reflectance + ambient_ir, epsilon)
```

The exact function can be simpler in code, but it should preserve the important
polarity:

```text
white / high reflection -> lower decay time
black / low reflection  -> higher decay time
```

The default policy observation is four normalized floats:

```text
obs = clamp((decay_time - qti_white_time) / (qti_black_time - qti_white_time), 0, 1)
0.0 ~= calibrated white / high reflection
1.0 ~= calibrated black / low reflection
```

The same 0-1 convention must be used on the ActivityBot. Robot-side Propeller C
should normalize each `rc_time` reading with the same polarity before calling the
policy:

```text
obs = clamp((qti_raw - qti_white_calibration) / (qti_black_calibration - qti_white_calibration), 0, 1)
```

Noise and mismatch should be applied before normalization:

```text
sensor_lateral_jitter_m
sensor_forward_jitter_m
sensor_gain
sensor_bias
sensor_noise_std
sensor_latency_steps
white_reflectance
black_reflectance
ambient_ir
qti_white_time
qti_black_time
qti_timeout
```

`sensor_gain` and `sensor_bias` are per-sensor episode parameters, not only
per-step noise. This models the real case where one QTI sensor is consistently
more or less sensitive than the others for an entire run.

The env may also compute a thresholded `qti_dark_bit` for debug rendering and
heuristic baselines:

```text
qti_dark_bit = obs >= threshold
```

That bit should match Parallax's documented digital polarity where black/no
reflection reads as `1` and white/reflection reads as `0`, but it should not be
the primary policy input for v1 unless raw RC readings prove too noisy on the
actual robot.

## Action Space

V1 should use two continuous actions because that best matches the real
differential-drive robot. The policy outputs normalized left and right wheel
commands:

```text
action[0] = left_wheel_command  in [-1.0, 1.0]
action[1] = right_wheel_command in [-1.0, 1.0]
```

The env clamps both values to `[-1, 1]`, then maps them to wheel speeds:

```text
v_left_cmd  = action[0] * max_wheel_speed_mps
v_right_cmd = action[1] * max_wheel_speed_mps
```

For PufferLib/Ocean binding this should follow the existing continuous-action
pattern used by `ocean/squared_continuous`:

```text
NUM_ATNS = 2
ACT_SIZES {1, 1}
```

The real robot adapter maps the same normalized actions to ActivityBot
`drive_speed(left_ticks_per_s, right_ticks_per_s)` or the correct lower-level
servo command for this ActivityBot hardware:

```text
left_ticks_per_s  = clamp(action[0], -1, 1) * max_ticks_per_s
right_ticks_per_s = clamp(action[1], -1, 1) * max_ticks_per_s
```

The exact speed range should be randomized per episode around nominal values:

```text
max_wheel_speed_mps
max_ticks_per_s
left_speed_scale
right_speed_scale
command_deadband
command_slew_limit
```

Discrete actions are the backup plan, not V1. If continuous training or
deployment fails, add a fallback action table that maps a small discrete action
id to left/right speed pairs.

## Reward

Reward should encourage forward progress along the line while keeping the robot
recoverable.

Per-step reward components:

```text
+ progress_delta * progress_scale
- centerline_reward or -centerline_distance_penalty from privileged true state
- heading_error_penalty from privileged track tangent
- lost_line_penalty when no sensor sees black
- action_smoothness_penalty for twitchy wheel commands
- reverse_penalty for excessive backward motion
```

Reward should use the true simulated state, not only what the QTI sensors can
observe. For example, if the line is between the two right sensors, the policy
may see ambiguous sensor values, but the reward should still score the robot by
its true distance to the centerline and true progress along the track. This
privileged reward shaping is allowed because reward is not deployed to the
ActivityBot.

Termination:

```text
episode_step >= max_steps
lost_line_steps >= lost_line_limit
robot too far from track centerline
track complete
```

The reward can use privileged distance/progress state because reward is not
deployed. The policy observation must remain restricted to what the robot can
read.

## Domain Randomization

Randomization is central to this env. The policy should not overfit exact sensor
spacing, exact wheel response, or exact marker appearance.

Initial randomized parameters:

```text
wheel_base_m
wheel_radius_scale_left
wheel_radius_scale_right
motor_lag_alpha
max_wheel_speed_mps
max_ticks_per_s
left_speed_scale
right_speed_scale
command_deadband
command_slew_limit
sensor_inner_x_m
sensor_outer_x_m
sensor_forward_y_m
sensor_lateral_jitter_m
sensor_forward_jitter_m
sensor_gain
sensor_bias
sensor_noise_std
sensor_latency_steps
white_reflectance
black_reflectance
ambient_ir
qti_white_time
qti_black_time
qti_timeout
sensor_failure_count
sensor_failure_mode
line_width_m
line_width_variation_m
line_edge_softness_m
line_reflectance_noise
effective_line_width_jitter_m
track_family
track_num_primitives
track_length_m
track_straight_length_min_m
track_straight_length_max_m
turn_radius_min_m
turn_radius_max_m
arc_angle_min_rad
arc_angle_max_rad
corner_mode
corner_fillet_radius_m
track_bounds_m
start_lateral_offset_m
start_heading_offset_rad
```

Randomization should start wide but not absurd. If training fails completely,
narrow the ranges until a policy learns, then widen them incrementally.

## Rendering

The raylib renderer should show:

- White floor.
- Black line.
- Robot body.
- Four sensor positions.
- Sensor positions, normalized QTI levels, and thresholded black/white debug
  state.
- Recent trajectory.
- Current action name.
- Episode reward/progress/lost-line count.

The render should be useful for debugging policy behavior, not visually ornate.

## PufferLib Layout

Target env files:

```text
config/line_follow.ini
ocean/line_follow/line_follow.h
ocean/line_follow/line_follow.c
ocean/line_follow/binding.c
ocean/line_follow/line_follow_track.h    optional if track code gets large
```

Existing support files:

```text
ocean/line_follow/firmware/
ocean/line_follow/host/
ocean/line_follow/scripts/
ocean/line_follow/docs/
```

The C env should follow common Ocean patterns as they are actually used by
Breakout, 2048, Boxoban/Sokoban-like envs, and Whisker Racer:

- `line_follow.h` should own the env structs, constants, helper math, reset,
  step, reward, observation fill, allocation/free helpers, and raylib render
  implementation. In the current Ocean style, most env logic lives in the
  header.
- Track generation can start in `line_follow.h` if it is small. If it starts to
  dominate the file, split it into `line_follow_track.h` and include that from
  `line_follow.h`, similar to how Boxoban uses helper headers for map handling.
  Track generation code must stay reset-time fast; do not put slow search or
  heap allocation in the reset path.
- `binding.c` should be small. It includes `line_follow.h`, declares
  observation/action/log shapes, defines `Env`, includes `vecenv.h`, and
  implements `my_init` and `my_log`.
- `line_follow.c` should be a standalone demo/human-play/eval harness, similar
  to `breakout.c`, `g2048.c`, and `whisker_racer.c`. It may include
  `puffernet.h`, load weights, run a local render loop, and allow keyboard
  control. It should not be the canonical env implementation.
- Keep helper math local and simple.
- Avoid dynamic allocation in the hot loop.
- Keep per-env state contiguous and cache-friendly.

Concrete templates:

- `ocean/breakout/breakout.h` contains the `Breakout` state, `init`,
  `allocate`, `c_reset`, `c_step`, helpers, and `c_render`; `breakout.c` is a
  short demo loop.
- `ocean/g2048/g2048.h` contains the `Game` state, reset/step/render, and
  board helpers; `g2048.c` is a small demo loop.
- `ocean/boxoban/boxoban.h` contains the Sokoban-like env state and
  reset/step/render; `binding.c` is only the adapter.
- `ocean/whisker_racer/whisker_racer.h` is useful to peek at for vehicle/env
  wiring and track rendering, but do not copy its track generator.

## Development And TDD

This env should be built test-first where practical. Before implementing a new
behavior, add the smallest useful test, run the test suite, confirm the new test
is the only expected failure, implement the behavior, then rerun the tests.

Tests should live under:

```text
ocean/line_follow/tests/
```

The first tests should focus on deterministic, easy-to-break behavior:

- QTI normalization maps calibrated white near `0.0` and calibrated black near
  `1.0`.
- Continuous actions are clamped to `[-1, 1]` and map to left/right wheel speeds
  with the expected polarity.
- Track templates generate in bounds, stay within fixed sample caps, and produce
  monotonically increasing progress.
- Sensor distance/coverage behaves correctly on a straight line, an arc, and an
  off-line point.
- Reward uses privileged true centerline distance/progress, not just thresholded
  sensor bits.
- Reset/step can run many iterations without crashes, NaNs, or dynamic
  allocation in the hot path.

Tests should be run regularly before commits. Exact harness details can follow
PufferLib's existing C/Python test conventions once implementation starts, but
the intent is to keep enough coverage that geometry, sensor polarity, and action
mapping regressions are caught early.

### Verified V1 Simulator Commands

Run from `/home/claude/sim2real` using this clone's `.venv`.

Focused tests and builds:

```bash
ocean/line_follow/tests/run_tests.sh
./build.sh line_follow --local
source .venv/bin/activate && \
  CCACHE_DIR=/home/claude/sim2real/.ccache \
  UV_CACHE_DIR=/home/claude/sim2real/.uv-cache \
  CUDA_HOME=/usr/local/cuda \
  ./build.sh line_follow
ASAN_OPTIONS=detect_leaks=0 ./line_follow --help
```

Native CUDA training and eval:

```bash
.venv/bin/python -m pufferlib.pufferl train line_follow --train.gpus 1

DISPLAY=:0 .venv/bin/python -m pufferlib.pufferl eval line_follow \
  --train.gpus 1 --load-model-path latest
```

Training must use the native backend, not `--slowly`, for the deployment path
that expects raw native fp32 checkpoints. In Codex, the sandbox may hide CUDA
devices; if training fails with `CUDA is not available`, rerun with GPU-visible
permissions rather than switching to the CPU backend. Do not add ad hoc
`--train.total-timesteps ...` overrides for line-follow probes; use the
checked-in `config/line_follow.ini` budget unless the user explicitly requests a
different budget. The eval command loops until the raylib window exits; for a
bounded render/load smoke check, prefix with `PYTHONUNBUFFERED=1 DISPLAY=:0
timeout 10s`.

The first default 20M-step V1 probe wrote
`logs/line_follow/1779931009464.json` and reached `env/perf=0.570`,
`env/score=0.826`, `env/centerline_error=0.011`, and about `2.0M` final SPS.

The first full 200M-step V1 train wrote `logs/line_follow/1779931480083.json`
and reached final `env/perf=0.944`, `env/score=1.234`,
`env/episode_return=3.14`, `env/episode_length=42.04`,
`env/centerline_error=0.0081`, and `SPS=1,635,391`. The latest checkpoint load
was verified on `DISPLAY=:0` from
`checkpoints/line_follow/1779931480083/0000000199753728.bin`.

## Training Plan

Training should start with easier curricula:

1. Straight lines with low randomization.
2. Gentle arc-only curves and S-curves.
3. Closed smooth loops such as ovals and rounded rectangles.
4. Wider sensor/drive/line-reflectance randomization.
5. More aggressive start offsets.
6. Held-out tracks from different primitive-generator seeds.
7. Later curriculum: radiused 90-degree turns, then sharp 90-degree turns.

Short-run acceptance checks:

```text
run line_follow tests
build native env successfully
run a random policy without crashes
render a heuristic controller following a straight/gentle curve
train a small policy that beats heuristic-free random behavior
eval policy on held-out randomized tracks
```

The first policy should be intentionally tiny. Deployment constraints matter
more than squeezing out max sim performance early.

## Sweep Constraints

Line-following sweeps must be deployment-aware. Do not inherit the broad default
PufferLib sweep range for model size. The default config allows
`policy.hidden_size` up to 1024 and `policy.num_layers` up to 8, which is useful
for larger games but wrong for a Parallax Propeller deployment target.

The sweep should restrict architecture first, then optimize reward/randomization:

```ini
[policy]
hidden_size = 16
num_layers = 1

[sweep.policy.hidden_size]
distribution = uniform_pow2
min = 8
max = 64
mean = 16
scale = auto

[sweep.policy.num_layers]
distribution = int_uniform
min = 0
max = 2
mean = 1
scale = auto
```

Initial deployment-focused sweeps should prefer:

- `hidden_size` in `{8, 16, 32}`.
- `hidden_size = 64` only as an upper-bound comparison.
- the normal PufferLib training stack unless a concrete export/deployment reason
  forces a change.
- feed-forward policies first because they are simplest to export to Propeller C.
- recurrent policies only if feed-forward policies fail, and only if the
  recurrent state can be represented and updated in simple C.

Every sweep candidate should be judged by both sim score and deployability:

```text
parameter_count
estimated C array size
estimated Propeller RAM/EEPROM footprint
estimated policy_forward latency at 20 Hz
```

A larger policy that trains slightly better in sim but cannot fit or run cleanly
on the ActivityBot is a failed candidate for this project.

After the base task works, evaluate a sensor-failure curriculum where each
episode may disable zero, one, or two sensors. This should be tracked separately
from v1 metrics so base sim2real progress does not get blocked by the stretch
goal.

## Deployment Plan

Robot deployment should preserve the same interface:

```text
qti_pins -> rc_time readings -> normalized obs[4] in fixed order -> policy_forward(obs) -> action[2]
action[2] -> clamp/scale -> drive_speed(left_ticks_per_s, right_ticks_per_s)
```

Policy export should prefer C arrays or a tiny binary generated from the trained
weights. The Propeller side should implement only the required forward pass.

Do not plan to run ONNX, Torch, NumPy, or a general model interpreter on the
Propeller.

## Open Measurement Work

Before final deployment tuning, measure or confirm:

- Real QTI pin mapping and physical order.
- Raw QTI RC-time ranges on white paper, black marker, black tape, and mixed
  edge cases.
- Whether the Propeller C deployment should use `rc_time(pin, 1)` raw readings,
  the Blockly-style 230 us fixed-timing digital read, or both.
- Per-sensor calibration values for white and black normalization.
- Sensor lateral spacing.
- Sensor forward offset from wheel axle.
- Wheelbase.
- ActivityBot ticks/second speed range that is reliable on paper.
- How fast the control loop can run while reading QTI and commanding wheels.
- Whether `abdrive.h` or `abdrive360.h` is the correct library for this specific
  robot hardware.

The sim can start before these are exact because randomization should cover a
range, but real measurements should become the nominal center values.

## V1 Definition Of Done

V1 is done when:

- `ocean/line_follow` builds as a native PufferLib C env.
- The env can train and render locally on g240.
- A trained small policy follows randomized held-out sim tracks.
- The policy input is exactly four normalized QTI RC-decay observations.
- The policy output is exactly two continuous normalized wheel-speed actions.
- There is a documented export path from PufferLib weights to robot-side C data.
- There is a robot-side harness that reads QTI sensors and executes the same
  0-1 observation normalization and continuous action scaling.
- Focused line-follow tests exist and pass.
