# Line Follow Best Runs

This file tracks physically relevant H4/L1 line-follow checkpoints. Do not pick
robot candidates from sim `perf` alone; use fixed-point replay and physical
outcomes as the final screen.

## Current Physical Contenders

| run | source | checkpoint | physical note | perf | checkpoint accuracy | progress | path efficiency | avg speed m/s | idle | lost line | complete |
|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `h85dqfhx` | `sim3` physical baseline | `checkpoints/line_follow/h85dqfhx/0000000084148224.bin` | Committed baseline. Roughly 80% there; slight strategic wobble, good responsiveness. | 0.527 | 0.567 | 0.628 | 0.986 | 0.199 | 0.058 | 0.334 | 0.667 |
| `bxycoua5` | `sim4` / `vivid-brook-1` | `checkpoints/line_follow/bxycoua5/0000000084148224.bin` | Strong physical contention with `h85dqfhx`; looked about as good on the real bot. | 0.529 | 0.573 | 0.633 | 0.987 | 0.198 | 0.064 | 0.328 | 0.673 |
| `0asku73p` | `sim4` / `cosmic-armadillo-7` | `checkpoints/line_follow/0asku73p/0000000079953920.bin` | Did well physically too, but left the track a couple times. Viable, likely less corrective on weak-center states. | 0.467 | 0.497 | 0.564 | 0.977 | 0.216 | 0.021 | 0.402 | 0.598 |
| `4mcnhp5u` | `sim4` / `magic-terrain-629` | `checkpoints/line_follow/4mcnhp5u/0000000139722752.bin` | Top `sim4` perf, but terrible physically; went straight off the track. | 0.607 | 0.647 | 0.722 | 0.984 | 0.201 | 0.044 | 0.234 | 0.766 |
| `sk0d7r7y` | `sim4` / `playful-glade-233` | `checkpoints/line_follow/sk0d7r7y/0000000131072000.bin` | Best deterministic benchmark of the top `sim4` cluster, but not physically tested and likely shares the bad weak-line bias. | 0.603 | 0.642 | 0.715 | 0.984 | 0.205 | 0.058 | 0.242 | 0.758 |

## Deploy-Sensitive Hypers

| run | max wheel m/s | deadband | min ticks | avg speed target | path eff weight | wasted motion | lost recovery | smoothness | lost limit | qti threshold | line noise |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `h85dqfhx` | 0.2889 | 0.0687 | 5 | 0.182 | n/a | n/a | n/a | 0.000 | 7 | 0.1746 | 0.3777 |
| `bxycoua5` | 0.2889 | 0.0687 | 5 | 0.182 | 0.300 | 0.015 | 0.200 | 0.004 | 7 | 0.1746 | 0.3777 |
| `0asku73p` | 0.3000 | 0.0521 | 6 | 0.164 | 0.000 | 0.016 | 0.073 | 0.004 | 8 | 0.1683 | 0.4050 |
| `4mcnhp5u` | 0.2892 | 0.0528 | 5 | 0.149 | 0.174 | 0.013 | 0.303 | 0.009 | 10 | 0.1500 | 0.3000 |
| `sk0d7r7y` | 0.3000 | 0.0459 | 5 | 0.140 | 0.190 | 0.007 | 0.306 | 0.012 | 10 | 0.1500 | 0.3615 |

## Fixed-Point Replay Screen

Values are mean left/right wheel ticks over 16 repeated fixed-point recurrent
steps. The parenthesized number is `right - left`; positive means left-turning
command, negative means right-turning command.

| run | all white | weak center | strong center | balanced weak | left black | right black |
|---|---:|---:|---:|---:|---:|---:|
| `h85dqfhx` | 35/5 (-30) | 60/91 (+31) | 91/91 (+0) | 91/67 (-24) | 5/91 (+86) | 91/5 (-86) |
| `bxycoua5` | 33/5 (-28) | 68/75 (+8) | 91/91 (+0) | 91/52 (-38) | 5/91 (+86) | 91/5 (-86) |
| `0asku73p` | 48/10 (-38) | 93/94 (+1) | 94/94 (+0) | 94/94 (-0) | 6/94 (+88) | 94/6 (-88) |
| `4mcnhp5u` | 5/75 (+70) | 12/91 (+79) | 87/91 (+4) | 31/91 (+60) | 5/91 (+86) | 91/5 (-86) |
| `sk0d7r7y` | 5/62 (+57) | 22/94 (+72) | 69/94 (+25) | 76/94 (+18) | 5/94 (+89) | 94/5 (-89) |

## Lessons

- `sim4` top `perf` is not sufficient for physical selection. `4mcnhp5u` had
  the best final `perf` but failed immediately on the real track.
- The fixed-point weak-observation sign screen predicted that risk. The bad
  top cluster turns the opposite way from the known-good physical baseline on
  all-white and weak/balanced observations.
- `bxycoua5` is valuable because it keeps the same recovery signs as
  `h85dqfhx` while using the new `sim4` objective. Its hypers also stayed close
  to the known-good deploy scaling: `max_wheel_speed_mps ~= 0.289`, deadband
  `~= 0.069`, min ticks `5`, lost-line limit `7`, and `qti_threshold ~= 0.175`.
- `0asku73p` is also physically viable, but its fixed-point replay goes nearly
  straight on `weak_center` and `balanced_weak`. That may explain why it looked
  good overall but still left the track a couple times: it avoids the bad
  opposite recovery sign, but may not correct hard enough before sharp turns.
- More aggressive `sim4` winners tended toward lower deadband, lower
  `avg_speed_perf_target_mps`, higher lost-line limit, lower `qti_threshold`,
  and stronger lost-line recovery shaping. In sim this improved progress and
  completion, but physically it can select the wrong lost-line search direction.
- For the next sweep, keep the new path-efficiency/wasted-motion idea, but add
  a selection gate or sweep constraint that preserves sane fixed-point recovery
  signs on `all_white`, `weak_center`, and `balanced_weak` before physical
  testing.

## Current EEPROM Notes

As of 2026-06-02, the last successful EEPROM image flashed for physical testing
was:

```text
ocean/line_follow/build/line-follow/line_follow_model_live_drive_p765_h4l1_h85dqfhx_video_30000ms_start2000ms_statusleds.elf
```

Recent alternate contenders were:

```text
ocean/line_follow/build/line-follow/line_follow_model_live_drive_p765_h4l1_0asku73p_sim4signok2_30000ms_start2000ms_statusleds.elf
```

```text
ocean/line_follow/build/line-follow/line_follow_model_live_drive_p765_h4l1_bxycoua5_sim4signok_30000ms_start2000ms_statusleds.elf
```

The rejected top-perf image was:

```text
ocean/line_follow/build/line-follow/line_follow_model_live_drive_p765_h4l1_4mcnhp5u_sim4top_30000ms_start2000ms_statusleds.elf
```
