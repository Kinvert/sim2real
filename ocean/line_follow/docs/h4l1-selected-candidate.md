# H4/L1 Selected Candidate

## 2026-06-01 Physical Baseline

Current selected checkpoint:

```text
project: sim3
run_id: h85dqfhx
checkpoint: checkpoints/line_follow/h85dqfhx/0000000084148224.bin
policy: hidden_size=4, num_layers=1
deploy: fixed-point recurrent Propeller C, p765 QTI pins [P7, P6, P5]
```

This is the first H4/L1 checkpoint that looked strongly viable on the physical
paper track. The robot still showed slight wobble when the line sat between two
sensors, but the wobble looked like strategic correction rather than runaway
oscillation. It was not delivery-perfect, but it was roughly 80% of the desired
behavior and much better than the earlier left-runaway candidates.

Run-matched deployment settings from `logs/line_follow/h85dqfhx.json`:

```text
dt=0.0165
dt_min=0.015
dt_max=0.020
max_wheel_speed_mps=0.2888961521163583
command_deadband=0.06867213468067349
min_drive_ticks_per_sec=5
qti_white_time=80
qti_black_time=403
qti_threshold=0.17455456797033547
```

Sim3 final summary:

```text
env/perf=0.5270
env/terminal_lost_line=0.334
env/terminal_track_complete=0.667
env/avg_forward_speed_mps=0.199
```

This run was not the highest sim-perf candidate. It was selected after physical
testing showed that higher-perf runs like `h4jou1ga`, `u14101qb`, and
`lqak2q8a` had a learned all-white / weak-center left-turn bias. On the real
robot, `h4jou1ga` wobbled, left the track to the left, and continued turning
left. The fixed-point recurrent diagnostic showed why:

```text
h4jou1ga all_white -> 5/65 ticks
h4jou1ga weak_center -> 17/93 ticks
```

That command pattern makes the right wheel much faster than the left, so the
robot keeps turning left after it loses the line. `h85dqfhx` instead produces:

```text
h85dqfhx all_white -> 37/5 ticks
h85dqfhx weak_center -> 42/25 ticks
```

This makes it recover in the opposite direction from the observed failure mode.
That tradeoff cost sim perf but produced much better physical behavior.

Repeatable default EEPROM build:

```bash
LINE_FOLLOW_FLASH_LOG=ocean/line_follow/build/line-follow/eeprom-h4l1-h85dqfhx.txt \
  ocean/line_follow/scripts/line-follow-h4l1-flash-deploy.sh /dev/ttyUSB0
```

The helper defaults now point to `h85dqfhx`. Override with `LINE_FOLLOW_RUN_ID`,
`LINE_FOLLOW_H4L1_SELECTED_RUN_ID`, or `LINE_FOLLOW_WEIGHTS` for fallback tests.
