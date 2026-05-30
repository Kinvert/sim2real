# PufferLib Line-Following Sim2Real Spec

This project is the line-following sim2real path for a Parallax ActivityBot with
a QTI Line Follower kit. The target real task is simple: drive on white paper and
follow black marker or tape lines using the currently selected three QTI sensors.

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

The current deployment policy consumes three normalized QTI readings ordered
`[left, middle, right]` from pins `[P7, P6, P5]`; P4/far-right is ignored on the
current branch. It chooses the same wheel-speed actions used in sim. The
robot-side code should not depend on Python, Torch, ONNX Runtime, or hosted OS
libraries.

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
- Three policy-facing QTI-like downward-facing sensors with analog-like RC decay
  readings ordered `[left, middle, right]`; the physically mounted P4/far-right
  sensor is ignored by this branch.
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
explicit current obs[3] -> policy_forward in simple C -> continuous action[2]
```

Do not add hidden env observations such as previous QTI readings unless they are
explicitly part of the observation and can be reproduced on the robot. If a
recurrent network is ever used, its recurrent state must be an explicit exported
model state that the Propeller C runner updates exactly; it is not a secret extra
environment input.

## Stretch Goals

Once the normal three-sensor policy works, add sensor-failure robustness. The env
should be able to randomize an episode where zero, one, two, or three QTI
sensors are failed. A failed sensor should look like a plausible real failure
mode, such as
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

Physical sensor order must be stable in sim and real. The current 3-sensor
policy observes:

```text
[left, middle, right]
```

If the real wiring ends up in a different physical order, robot-side code should
reorder the pins before telemetry or before selecting the policy inputs. The
current firmware default pin mapping is:

```text
left   = P7
middle = P6
right  = P5
```

P4 / far-right is intentionally ignored on the current branch. The policy
should not need to know board pin numbers.

## Measured ActivityBot Geometry

Robot pose is modeled at the center of the drive axle. Body-frame coordinates
use `forward_m` along the robot heading and `lateral_m` positive to the robot's
left.

Measured dimensions from the current robot:

```text
tire-to-tire width:               125 mm = 0.125 m
tire diameter:                     65 mm = 0.065 m
body behind axle:                  90 mm = 0.090 m
body ahead of axle:                40 mm = 0.040 m
total body length around axle:    130 mm = 0.130 m
QTI sensor forward offset:       43.5 mm = 0.0435 m ahead of axle
side QTI lateral offsets:         +/-18 mm = +/-0.018 m from robot centerline
middle QTI lateral offset:          0 mm = 0.000 m from robot centerline
left-to-right policy spread:       36 mm = 0.036 m
```

Nominal QTI body-frame positions:

```text
left    = (forward_m = 0.0435, lateral_m =  0.018)
middle  = (forward_m = 0.0435, lateral_m =  0.000)
right   = (forward_m = 0.0435, lateral_m = -0.018)
```

The sim and renderer should treat these as the nominal geometry before applying
episode randomization. If the "tire-to-tire width" later proves to be measured
between outer tire faces rather than wheel contact centerlines, update
`wheel_base_m` separately; the differential-drive model needs center-to-center
wheel contact spacing.

Current retrain defaults randomize lateral QTI positions by `0.004 m` and
forward positions by `0.003 m` to reflect measured build imprecision without
overwhelming the nominal sensor geometry. Sensor failure randomization is
intentionally left out of the next retrain.

The current top-down renderer still uses visual placeholders for unmeasured
widths: `body_width_m = 0.075` and `tire_width_m = 0.012`. These affect only
the debug drawing, not the kinematics or QTI sensor positions.

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
edge softness and reflectivity variation. The default hand-drawn stripe model is
an 18 mm nominal line, randomized per episode by +/-6 mm, with an additional
smooth per-segment width wobble of +/-2 mm along the sampled track.

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

For the current random training generator, pure straight tracks are excluded
because a saturated forward policy can score by chance. Straight tracks remain
available as explicit test/eval/smoke cases.

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
line_width_jitter_m
line_width_segment_jitter_m
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

Current assumed sim control rate for the smaller model:

```text
dt = 0.0065 seconds
dt_min = 0.004 seconds
dt_max = 0.009 seconds
```

This is not a pause inserted into PufferLib. It is the distance-integration time
between policy decisions: how far the robot moves before the next observation
and action. Live host-timestamp telemetry of the exported Propeller model, built
with `LINE_FOLLOW_LOOP_MS=25` and `LINE_FOLLOW_PRINT_EVERY=50`, measured an
effective closed-loop update period of about 476.6 ms. That includes QTI reads,
model inference, any drive command, amortized serial output, and the requested
Propeller pause.

During training, the env samples an episode-level `episode_dt` from
`[dt_min, dt_max]` and uses that value for kinematic integration. The previous
hidden-size-16 recurrent exported model measured about 476.6 ms/update; the
hidden-size-8 recurrent model measured about 187.5 ms/update with the same
requested 25 ms Propeller pause. The old three-observation H8/L0 float path
measured about 24.0 ms/update with `LINE_FOLLOW_LOOP_MS=0` and amortized serial
telemetry. After folding the feed-forward action head to a 2x3 fixed-point
policy, the real robot measured about 6.4 ms/update with motors enabled, no SD
logging, and sparse host-timestamp telemetry. That H8/L0 shape is the current
default because the middle sensor removes the all-white centered-state
ambiguity that broke the two-sensor feed-forward policy.

The env applies model-aware timing before init when `model_dt_enabled = 1`.
PufferLib syncs the selected `[policy] hidden_size` and `num_layers` into
`[env] policy_hidden_size` and `policy_num_layers`; `line_follow` then estimates
the real control period from that model shape. Current estimates are:

```text
H2/L0  -> 6.5 ms
H4/L0  -> 6.5 ms
H8/L0  -> 6.5 ms
H2/L1  ->  70 ms
H4/L1  -> 105 ms
H8/L1  -> 188 ms
H16/L1 -> 477 ms
```

When derived `dt` changes, `max_steps` and `lost_line_limit` are scaled
inversely so larger/slower models do not receive extra wall-clock episode time
or lost-line grace. The current `max_wheel_speed_mps = 0.116` is the doubled
ActivityBot speed window: `action=0.5` is about `18 ticks/s`, and `action=1.0`
is about `36 ticks/s`. At the default H8/L0 fixed-point measured `dt=0.0065`,
full-speed travel is about `0.75 mm` per policy decision.
`min_drive_ticks_per_sec = 6` applies a small anti-stall floor to policy-driven
wheel commands, about `19.1 mm/s` with the measured 65 mm tires. This uses the
first smooth slow drive probe; `2 ticks/s` moved inconsistently, `3 ticks/s`
produced repeatable but weak encoder movement, and `6 ticks/s` was smoother.
Sweeps may choose a fixed minimum floor per whole training run; logged `perf`
favors higher floors up to `20 ticks/s`.
`progress_reward_scale = 1.0`, `time_penalty = 0.005`, and
`idle_penalty = 0.02` keep centered forward progress positive while making
stopped or crawling policies lose reward. The env records centerline quality
every `centerline_reward_interval_m = 0.010` meters for debug, but training
reward is continuous per tick rather than a checkpoint bonus. It also penalizes
low average wheel command. With
`min_wheel_action = 0.35`, a pivot with one wheel stopped must drive the outside
wheel at roughly 70% command to avoid the idle penalty.
`turn_speed_penalty_scale = 0.05` adds a smaller privileged penalty when a
corrective turn is needed but the desired outside wheel is below
`min_turn_outer_action = 0.65`.
`firmware_loop_ms = 0` means the Propeller loop does not request an extra
post-inference pause; real control period is then
determined by QTI reads, model inference, drive calls, and serial printing.
Drive-enabled firmware builds default `LINE_FOLLOW_MAX_LOOPS = 0`, which means
no loop cap; the robot keeps running until reset or power-off unless a caller
explicitly sets `LINE_FOLLOW_MAX_LOOPS`.

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
- Do not install the kit's 10 kOhm resistors for the Propeller line-following
  path. The QTI Line Follower AppKit guide says those resistors can be added
  across QTI W and R to make an analog voltage output for A/D inputs, and
  explicitly warns not to use them with Parallax line-following example code.
  Our robot code uses the Propeller I/O RC-time/digital timing path, not an ADC
  voltage input.
- The optional resistor mentioned in Parallax's ActivityBot QTI test lesson is
  a 220 ohm resistor in the 5 V feed to reduce IR brightness if shiny paper or
  tape overwhelms the sensors. Do not add it for the first telemetry dump; only
  try it if the raw readings show saturation.

## Sensor Model

There are three policy-facing downward QTI sensors rigidly attached to the
robot. Sensor world positions are computed from robot pose and randomized
body-frame offsets. The far-right/P4 sensor may still be physically mounted, but
it is ignored by the current policy and firmware.

Nominal body-frame layout uses the measured axle-centered coordinates above:

```text
left    forward = sensor_forward_m, lateral =  sensor_side_lateral_m
middle  forward = sensor_forward_m, lateral =  0
right   forward = sensor_forward_m, lateral = -sensor_side_lateral_m
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

Each policy-facing physical sensor is normalized to `0..1`. The default policy
observation is the three normalized floats:

```text
obs = clamp((decay_time - qti_white_time) / (qti_black_time - qti_white_time), 0, 1)
0.0 ~= calibrated white / high reflection
1.0 ~= calibrated black / low reflection
```

The same 0-1 convention must be used on the ActivityBot. Robot-side Propeller C
should normalize each `rc_time` reading with the same polarity, then pass the
left, middle, and right values to the policy:

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
left_unit   = max(clamp(action[0], -1, 1), 0)
right_unit  = max(clamp(action[1], -1, 1), 0)
v_left_cmd  = left_unit * max_wheel_speed_mps
v_right_cmd = right_unit * max_wheel_speed_mps
if policy drive is enabled:
    v_left_cmd  = max(v_left_cmd, min_drive_ticks_per_sec converted to m/s)
    v_right_cmd = max(v_right_cmd, min_drive_ticks_per_sec converted to m/s)
```

This branch trains non-reversing wheel commands. Negative actions are allowed by
the policy interface but are clamped to a non-reversing wheel and penalized
rather than used as reverse by default. With the default anti-stall floor, a
policy command below the deadband still maps to `6 ticks/s` during live drive;
explicit startup/shutdown calls still use `drive_speed(0, 0)`. The idle penalty
is based on average commanded wheel speed, so a tiny inside-wheel floor is still
treated as crawling when the outside wheel is not moving enough.

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
left_ticks_per_s  = 0.5 * (clamp(action[0], -1, 1) + 1) * max_ticks_per_s
right_ticks_per_s = 0.5 * (clamp(action[1], -1, 1) + 1) * max_ticks_per_s
```

The exact speed range should be randomized per episode around nominal values:

```text
max_wheel_speed_mps
max_ticks_per_s
left_speed_scale
right_speed_scale
command_deadband
min_drive_ticks_per_sec
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
+ normalized per-tick true progress * reward_centerline_score * progress_reward_scale
- optional centerline_distance_penalty from privileged true state
- heading_error_penalty from privileged track tangent
- lost_line_penalty when privileged geometry says the line left the sensor span
- time_penalty every tick to discourage crawling or stopping
- idle_penalty when average post-deadband wheel command is below threshold
- turn_speed_penalty when the desired outside wheel is too slow during a
  privileged corrective turn
- negative wheel commands clamp that wheel to the non-reversing floor and receive
  reverse_penalty_scale proportional to the negative command amount
+ success_reward on timeout or track completion only if the robot stayed on
  the line, never idled, and scaled by the episode progress-times-accuracy perf
```

The final reward emitted to PufferLib is clipped to `[-1, 1]` after all shaping
and terminal bonus terms are applied.

`accuracy_score` is a simple millimeter-scale centerline score:

```text
1.0 at 0 mm centerline error
0.833 at 5 mm
0.667 at 10 mm
0.500 at 15 mm
0.333 at 20 mm, where a measured inner sensor is directly over the stripe center
0.000 at 30 mm and beyond
```

With side sensors at +/-18 mm and an 18 mm nominal stripe, 30 mm is the
zero-quality point for centerline accuracy. Lost-line termination remains based
on privileged sensor-span geometry and may allow a few extra millimeters because
line width, edge softness, and sensor placement are randomized.

For reward shaping, `reward_centerline_score` is steeper than the logged
accuracy metric and is signed/clipped:

```text
1.0 at 0 mm centerline error
 0.0 at 15 mm centerline error
-1.0 at 30 mm centerline error and beyond
```

`perf` is a bounded 0..1 model-selection metric based on continuous effective
progress. Each positive centerline-progress delta contributes:

```text
effective_progress_m += track_progress_delta_m * accuracy_score
base_perf = effective_progress_m / target_progress_m, clipped to 0..1
min_drive_speed_score = min_drive_ticks_per_sec / 20, clipped to 0..1
perf = base_perf * min_drive_speed_score
```

`target_progress_m` is the smaller of track length and the physical distance
available at max wheel speed over the episode horizon. Debug checkpoint counts
still exist for telemetry, but they no longer pay reward. Training reward uses
the signed score so blind forward progress after losing the line becomes
negative rather than merely unrewarded. The minimum-speed multiplier is for
sweep/model selection only; terminal success bonuses still use `base_perf` so
changing the run-level anti-stall floor does not silently reshape reward.

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
min_drive_ticks_per_sec
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
line_width_jitter_m
line_width_segment_jitter_m
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
Current reset sampling chooses among the current episode's three jittered sensor
lateral targets plus centered, so some episodes begin with the black line under
the left or right policy sensor rather than only at the center sensor.

## Rendering

The raylib renderer should show:

- White floor.
- Black line.
- Robot body with the drive axle at the simulated pose, 90 mm of body behind
  the axle, and 40 mm of body ahead of it.
- Tires/wheels as top-down black bars, placed from the measured 125 mm
  tire-to-tire width. The measured 65 mm tire diameter is the fore/aft length
  of each bar in the top-down view.
- Three policy sensor positions using the measured 45 mm forward offset and
  lateral offsets `[+20 mm, 0 mm, -20 mm]`. P4/far-right may be physically
  mounted but is not part of the current policy/render contract.
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

.venv/bin/python -m pufferlib.pufferl sweep line_follow \
  --sweep.gpus 1 --train.gpus 1 --sweep.max-runs 1000 \
  --wandb --wandb-project sim1

DISPLAY=:0 .venv/bin/python -m pufferlib.pufferl eval line_follow \
  --train.gpus 1 --load-model-path latest
```

Training must use the native backend, not `--slowly`, for the deployment path
that expects raw native fp32 checkpoints. In Codex, the sandbox may hide CUDA
devices; if training fails with `CUDA is not available`, rerun with GPU-visible
permissions rather than switching to the CPU backend. Do not add ad hoc
`--train.total-timesteps ...` overrides for line-follow probes; use the
checked-in `config/line_follow.ini` budget unless the user explicitly requests a
different budget. The current standard budget is 100M timesteps; reserve 200M
for explicit long polish runs. Sweeps should also use the checked-in training
budget unless explicitly told otherwise, and the current sweep pins the deployed
H8/L0 policy shape while varying training, reward, and control values. The eval
command loops until the raylib window exits; for a
bounded render/load smoke check, prefix with `PYTHONUNBUFFERED=1 DISPLAY=:0
timeout 10s`.

`pufferl eval` uses the normal rollout path. For continuous policies that path
samples from the Gaussian policy distribution, so the raylib window is useful
for visual smoke testing but is not a deterministic deployment evaluation. Use
the benchmark and model-smoke tools below when judging whether a checkpoint is
actually deployable.

The first default 20M-step V1 probe wrote
`logs/line_follow/1779931009464.json` and reached `env/perf=0.570`,
`env/score=0.826`, `env/centerline_error=0.011`, and about `2.0M` final SPS.

An early full 200M-step V1 train wrote `logs/line_follow/1779931480083.json`
and reached final `env/perf=0.944`, `env/score=1.234`,
`env/episode_return=3.14`, `env/episode_length=42.04`,
`env/centerline_error=0.0081`, and `SPS=1,635,391`. That checkpoint load
was verified on `DISPLAY=:0` from
`checkpoints/line_follow/1779931480083/0000000199753728.bin`.

The current measured-control 200M-step train wrote
`logs/line_follow/1779950360328.json` with `dt=0.477`, `dt_min=0.450`,
`dt_max=0.520`, and `max_wheel_speed_mps=0.020`. It reached
`env/perf=0.334`, `env/score=0.623`, `env/episode_return=-0.418`,
`env/episode_length=10.48`, `env/centerline_error=0.0222`, and
`SPS=1,972,307`. Its checkpoint is
`checkpoints/line_follow/1779950360328/0000000199753728.bin`. Host smoke showed
the one-hot `outer_right` synthetic case turning left, so treat this checkpoint
as speed-scaled but not yet closed-loop drive-approved.

The hard-no-reverse 200M-step train wrote `logs/line_follow/1779952610219.json`
with `policy.hidden_size=8`, `policy.num_layers=0`, `dt=0.120`,
`dt_min=0.080`, `dt_max=0.200`, and `max_wheel_speed_mps=0.010`. It reached
final `env/perf=0.297`, `env/score=0.591`, `env/episode_return=-1.300`,
`env/episode_length=60.10`, `env/centerline_error=0.0087`, and
`SPS=2,129,033`. Its checkpoint is
`checkpoints/line_follow/1779952610219/0000000199753728.bin`. Bounded
`DISPLAY=:0` eval loaded the checkpoint successfully. Host smoke still shows
some negative raw actions on synthetic one-hot cases; current sim versions
clamp negative wheel commands to non-reversing wheel speed and penalize the
negative command amount. This old checkpoint predates the anti-stall floor;
treat it as not drive-approved.

That run predates the measured fixed-point three-sensor timing target. Retrain
before treating the current `dt=0.0065` / architecture sweep config as
evaluated.

The first `sim1` sweep after shrinking the deployable model plateaued near
`env/perf=0.33`. Investigation found two measurement issues: reset was crediting
absolute nearest-track progress on closed loops, and the random track lengths
were large relative to the 1 cm/s robot speed and fixed wall-clock horizon. The
current metric starts episode progress at zero, bounds per-step progress by
physical wheel travel, and measures `perf` from continuous effective progress.
Each positive track-progress delta is multiplied by the current
`accuracy_score`. Untraveled target distance counts as zero, so a short-lived
episode cannot get `perf=1.0` just because its starting pose was centered. If
`0.30 m` is physically possible during the episode, traveling `0.15 m` with
average accuracy `0.8` gives `perf = 0.15 * 0.8 / 0.30 = 0.4`.
The highest-reward state is centered over the line, with the middle sensor over
the stripe and symmetric side-sensor context. Lost-line
termination uses privileged simulator geometry: the true stripe must leave the
actual sensor span, independent of noisy or thresholded sensor observations.
The follow-up failure mode was centered stopping: a policy could park on the
track and avoid most penalties. The reward now pays accuracy-scaled progress,
applies a time penalty every tick, and applies an idle penalty unless average
commanded wheel speed is above the small motion threshold. Timeout/track-complete
success bonuses are scaled by the same progress-times-accuracy `perf`, so a
centered robot that has not moved cannot earn a survival bonus.

## Training Plan

Training should start with easier curricula:

1. Gentle arc-only curves and S-curves.
2. Closed smooth loops such as ovals and rounded rectangles.
3. Explicit straight-line smoke/eval cases only, not random training wins.
4. Wider sensor/drive/line-reflectance randomization.
5. More aggressive start offsets.
6. Held-out tracks from different primitive-generator seeds.
7. Later curriculum: radiused 90-degree turns, then sharp 90-degree turns.

Random training starts are clamped to `start_lateral_offset_m = 0.025` meters so
the line starts inside the active inner-sensor span. Heading starts randomize up
to `start_heading_offset_rad = 0.261799` radians, about 15 degrees.

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

The current ActivityBot defaults are even smaller:

```ini
[policy]
hidden_size = 8
num_layers = 0

[env]
dt = 0.0065  # measured folded fixed-point H8/L0 drive loop without SD logging
dt_min = 0.004
dt_max = 0.009
policy_hidden_size = 8
policy_num_layers = 0
model_dt_enabled = 1

[sweep.policy.hidden_size]
min = 2
max = 8
mean = 4

[sweep.policy.num_layers]
not swept by default; keep num_layers = 0 unless recurrence is deliberately tested
```

With `model_dt_enabled = 1`, those reference values keep the default H8/L0
episode range at `dt = 0.0065`, `dt_min = 0.004`, and `dt_max = 0.009`, while
recurrent policies get their own estimated timing. Feed-forward policies share
the same deployment timing because the firmware folds them to a fixed 2x3
action policy.

Initial deployment-focused sweeps should prefer:

- `hidden_size` in `{2, 4, 8}` while the policy must run on the Propeller.
- larger hidden sizes only as upper-bound comparisons.
- the normal PufferLib training stack unless a concrete export/deployment reason
  forces a change.
- H4/L0 and H2/L0 are useful timing lower bounds for the three-observation
  policy, but they must beat the H8/L0 deterministic benchmark before deployment.
- recurrent policies are timing-penalized candidates only; the current default
  is feed-forward H8/L0.
- every candidate must use its estimated `dt`; do not compare different model
  sizes at the same kinematic update period.

Every sweep candidate should be judged by both sim score and deployability:

```text
parameter_count
estimated C array size
estimated Propeller RAM/EEPROM footprint
estimated policy_forward latency at the measured/target control cadence
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
qti_pins[3] -> rc_time readings -> normalized obs[left, middle, right] -> policy_forward(obs) -> action[2]
action[2] -> clamp/scale -> drive_speed(left_ticks_per_s, right_ticks_per_s)
```

Policy export should prefer C arrays or a tiny binary generated from the trained
weights. The Propeller side should implement only the required forward pass.

Do not plan to run ONNX, Torch, NumPy, or a general model interpreter on the
Propeller.

## Host Model Smoke Test

Before QTI sensors are attached, smoke-test a trained checkpoint by feeding
synthetic normalized QTI observations into the native C inference path:

```text
ocean/line_follow/scripts/line-follow-model-smoke.sh
```

The script builds `ocean/line_follow/host/line_follow_model_smoke.c`, loads the
latest `checkpoints/line_follow/**/*.bin` checkpoint by default, and prints:

```text
obs[3] -> raw continuous action[2] -> clamped m/s -> approximate ticks/second
```

It uses the deployment geometry/scaling assumptions:

```text
action [-1, 1] -> [0, 0.116] m/s
approximately [0, 36] ActivityBot ticks/s
action 0.0 -> 0 ticks/s
action 0.5 -> about 18 ticks/s
action 1.0 -> about 36 ticks/s
65 mm tire diameter
64 encoder ticks / wheel revolution
```

To test one hand-written observation:

```text
ocean/line_follow/scripts/line-follow-model-smoke.sh 0.7 0.1 0.0
```

Each row resets the native MinGRU state so synthetic cases are independent. A
real robot loop must preserve recurrent state across timesteps if using the
native checkpoint directly.

## Propeller C Model Fake-Observation Test

To prove the trained model itself can run on the ActivityBot before QTI sensors
are attached, use the Propeller C model fake-observation firmware:

```text
ocean/line_follow/firmware/line_follow_model_fake.c
```

The build script chooses the newest `checkpoints/line_follow/**/*.bin` by
default. For a specific model, set `LINE_FOLLOW_WEIGHTS=/path/to/checkpoint.bin`.
It generates an embedded C header, compiles the Propeller C firmware, and prints
the checkpoint path at runtime:

```text
ocean/line_follow/scripts/parallax-build-line-follow-model-fake.sh
timeout 20s ocean/line_follow/scripts/parallax-run-line-follow-model-fake.sh /dev/ttyUSB0
```

This firmware runs on the robot. It does not read QTI pins, does not command
motors, and does not write EEPROM. It injects canned normalized observations,
runs the embedded trained PufferLib model forward pass in Propeller C, and
prints:

```text
M name left middle right action0 action1 left_ticks right_ticks
```

Old two-observation and four-observation checkpoints are intentionally
incompatible after the three-sensor switch. Re-run training before using the
host or Propeller model smoke tools; the shape check should fail if an old
checkpoint is accidentally selected.

Earlier Propeller fake-observation runs matched host-side native C smoke for the
same checkpoint and cases, confirming the checkpoint format, weight alignment,
MinGRU single-step recurrence, decoder output, action clamp/deadband, and wheel
tick scaling on the robot. Re-run the Propeller fake-observation test for any
new checkpoint before enabling drive.

## Deterministic Policy Benchmark

For repeatable host-side policy scoring, use the deterministic benchmark:

```text
ocean/line_follow/scripts/line-follow-policy-benchmark.sh
```

It builds `ocean/line_follow/host/line_follow_policy_benchmark.c`, loads the
latest checkpoint by default, uses the native `forward_puffernet` mean action,
and runs fixed track/start cases. Select suites with `--suite standard`,
`--suite stress`, `--suite seeded_random`, or `--suite all`. Use `--json PATH`
to save the same run in machine-readable form. The wrapper canonicalizes the
checkpoint with `realpath`; the text header and JSON both include
`checkpoint_path` so a benchmark artifact can be used directly for robot
deployment. The JSON also stores the model shape plus compact env, geometry,
QTI, and reward hyperparameter snapshots. The output is CSV-like and includes:

```text
case,steps,perf,progress_frac,track_progress_m,effective_progress_m,
target_progress_m,center_path_m,path_efficiency,avg_raw_left,avg_raw_right,
avg_cmd_left_mps,avg_cmd_right_mps,avg_speed_frac,idle_frac,all_white_frac,
visible_frac,lost,timeout,complete,negative,action_bound,success,reward
```

Useful deterministic timing/control probes:

```text
ocean/line_follow/scripts/line-follow-policy-benchmark.sh --dt 0.025
ocean/line_follow/scripts/line-follow-policy-benchmark.sh --motor-lag-alpha 1.0
ocean/line_follow/scripts/line-follow-policy-benchmark.sh --dt 0.025 --motor-lag-alpha 1.0
ocean/line_follow/scripts/line-follow-policy-benchmark.sh --suite all --json ocean/line_follow/build/line-follow/benchmark.json
```

Recent benchmark smoke for the measured-dt H8/L0 three-sensor checkpoint
`checkpoints/line_follow/6hsze6nn/0000000099876864.bin`: deterministic model
all-suite perf is `0.798560`, progress fraction is `0.890772`, effective
progress is `0.264144 m`, and average speed fraction is `0.746907`. The oval
cases still expose low-speed/stopping behavior, so use the benchmark suite as a
regression gate before treating a checkpoint as faster or smoother than the
current base.

## Propeller C Live QTI Model Telemetry

After the QTI sensors are attached, use live telemetry mode before enabling any
drive output:

```text
ocean/line_follow/firmware/line_follow_model_live.c
```

This firmware runs on the robot. It reads the real three active QTI sensors,
normalizes them, feeds `[left, middle, right]` into the embedded trained model,
prints raw/min/max/sensor/model/action/ticks/timing, and does not command motors
unless built with `--drive`. P4/far-right is ignored in this branch.

Build and run the telemetry-only version:

```text
ocean/line_follow/scripts/parallax-build-line-follow-model-live.sh
timeout 180s ocean/line_follow/scripts/parallax-run-line-follow-model-live.sh --log ocean/line_follow/build/line-follow/model-live-last.txt /dev/ttyUSB0
```

Analyze the captured data:

```text
ocean/line_follow/scripts/analyze-line-follow-live-log.py ocean/line_follow/build/line-follow/model-live-last.txt
```

The live row format is:

```text
L step raw0 raw1 raw2 min0 min1 min2 max0 max1 max2 sensor0 sensor1 sensor2 model0 model1 model2 action0 action1 left right dt_ms
```

For loop timing measurements, run the loader with `--host-timestamps`. The host
then prefixes each `L` row with a high-resolution timestamp, and
`analyze-line-follow-live-log.py` reports `host_loop_ms` by dividing elapsed host
time by telemetry step deltas. Current three-sensor firmware also emits a
trailing Propeller body-time `dt_ms` field; verify it against host timestamps
after timing-related firmware edits.

The QTI polarity assumption is that lower raw RC-time values are
brighter/whiter and higher raw values are darker/blacker. Current normalization
constants are:

```text
QTI_WHITE_TIME=40
QTI_BLACK_TIME=350
QTI_THRESHOLD_Q1000=500
```

The Propeller model-live and sanity build scripts read their default
`QTI_WHITE_TIME` and `QTI_BLACK_TIME` values from `config/line_follow.ini`, so
the firmware and trainable sim use the same normalization constants by default.
Override these at build time only for targeted hardware tests:

```text
QTI_WHITE_TIME=... QTI_BLACK_TIME=... ocean/line_follow/scripts/parallax-build-line-follow-model-live.sh
```

First real telemetry dump result:

```text
rows=399
raw min/max:
  outer_left   23..2563
  inner_left   15..2256
  inner_right  26..1748
  outer_right  23..1700
raw p05/p50/p95:
  outer_left   42/46/328
  inner_left   26/34/252
  inner_right  38/42/269
  outer_right  39/41/230
```

The old global `QTI_WHITE_TIME=100` compressed white readings for this hardware.
The p05/p95 values below are useful range targets, not fixed per-sensor constants
to bake into a model:

```text
white = [42, 26, 38, 39]
black = [328, 252, 269, 230]
```

The trainable sim should treat those values as evidence for domain
randomization. Current retrain defaults use nominal `qti_white_time=40` and
`qti_black_time=350`, then randomize actual per-sensor response each episode
with `qti_white_jitter=25` and `qti_black_jitter=120`. Observations are always
clamped after normalization, so the policy never receives values outside
`[0, 1]`. The Propeller side uses the same clamp formula:
`clamp((raw - white) / (black - white), 0, 1)`.

The run also produced timeout/off-paper style rows where all sensors jumped high
at once, for example `[2563, 2256, 1748, 1700]`. Treat those separately from
normal black-line calibration.

The drive-enabled build exists for later staged tests only:

```text
ocean/line_follow/scripts/parallax-build-line-follow-model-live.sh --drive
ocean/line_follow/scripts/parallax-run-line-follow-model-live.sh --drive /dev/ttyUSB0
```

## Propeller C Sanity Harness

Before perfecting the learned model, use the Propeller C sanity harness to test
the robot-side interface directly:

```text
ocean/line_follow/firmware/line_follow_sanity.c
```

With no QTI sensors attached, use fake-observation mode first. This mode does
not read QTI pins and does not command motors; it feeds canned normalized
observations through the Propeller C policy stub and prints the resulting wheel
commands:

```text
ocean/line_follow/scripts/parallax-build-line-follow-sanity.sh --fake
timeout 20s ocean/line_follow/scripts/parallax-run-line-follow-sanity.sh --fake /dev/ttyUSB0
```

The fake-observation output table is:

```text
F name left middle right left_ticks right_ticks
F all_white 0.000 0.000 0.000 12 -12
F all_black 1.000 1.000 1.000 20 20
F left 1.000 0.000 0.000 2 38
F middle 0.000 1.000 0.000 20 20
F right 0.000 0.000 1.000 38 2
F soft_left 0.800 0.200 0.000 6 34
F soft_right 0.000 0.200 0.800 34 6
F balanced_edge 0.200 1.000 0.200 20 20
```

The default build is telemetry-only. It reads three QTI sensors with
`high(pin); pause(1); rc_time(pin, 1)`, normalizes each reading to the sim
polarity as integer `0..1000`, runs a small policy-shaped stub, and prints:

```text
S raw0 raw1 raw2 obs0 obs1 obs2 left right
```

Default sensor order and pins are:

```text
obs[0] left    P7
obs[1] middle  P6
obs[2] right   P5
```

Override pin or calibration values at build time with environment variables:

```text
QTI_LEFT_PIN=...
QTI_MIDDLE_PIN=...
QTI_RIGHT_PIN=...
QTI_WHITE_TIME=...
QTI_BLACK_TIME=...
QTI_THRESHOLD_Q1000=...
```

Verified build commands from `/home/claude/sim2real`:

```text
ocean/line_follow/scripts/parallax-build-line-follow-sanity.sh
ocean/line_follow/scripts/parallax-build-line-follow-sanity.sh --fake
ocean/line_follow/scripts/parallax-build-line-follow-sanity.sh --drive
```

The telemetry-only ELF is about 9.3 KB. The fake-observation ELF is about 9.7
KB. The `--drive` ELF links `abdrive.h` and is about 16.6 KB. The drive-enabled
build is intentionally explicit and defaults to a bounded run; use it only with
the robot safely staged.

RAM-only load commands for the next hardware session:

```text
timeout 20s ocean/line_follow/scripts/parallax-run-line-follow-sanity.sh --fake /dev/ttyUSB0
ocean/line_follow/scripts/parallax-run-line-follow-sanity.sh /dev/ttyUSB0
ocean/line_follow/scripts/parallax-run-line-follow-sanity.sh --drive /dev/ttyUSB0
```

These wrappers use `propeller-load -r -t` and do not write EEPROM.

## Open Measurement Work

Before final deployment tuning, measure or confirm:

- Real QTI pin mapping and physical order.
- Raw QTI RC-time ranges on white paper, black marker, black tape, and mixed
  edge cases.
- Whether the Propeller C deployment should use `rc_time(pin, 1)` raw readings,
  the Blockly-style 230 us fixed-timing digital read, or both.
- Per-sensor calibration values for white and black normalization.
- Whether the measured 125 mm tire-to-tire width is wheel contact centerline
  spacing or outer tire-face spacing.
- Body width and top-down tire/tread width if render proportions need to match
  the physical chassis more closely.
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
- The policy input is exactly three normalized QTI RC-decay observations:
  `[left, middle, right]`.
- The policy output is exactly two continuous normalized wheel-speed actions.
- There is a documented export path from PufferLib weights to robot-side C data.
- There is a robot-side harness that reads QTI sensors and executes the same
  0-1 observation normalization and continuous action scaling.
- Focused line-follow tests exist and pass.
