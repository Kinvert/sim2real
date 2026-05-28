#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../line_follow.h"

static int failures = 0;

static void expect_true(bool value, const char* name) {
    if (!value) {
        fprintf(stderr, "FAIL: %s\n", name);
        failures++;
    }
}

static void expect_near(float actual, float expected, float eps, const char* name) {
    if (fabsf(actual - expected) > eps) {
        fprintf(stderr, "FAIL: %s actual=%f expected=%f eps=%f\n", name, actual, expected, eps);
        failures++;
    }
}

static LineFollow make_test_env(float* obs, float* actions, float* rewards, float* terminals) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.observations = obs;
    env.actions = actions;
    env.rewards = rewards;
    env.terminals = terminals;
    env.num_agents = 1;
    init(&env);
    return env;
}

static void test_qti_normalization(void) {
    expect_near(qti_normalize(100.0f, 100.0f, 1100.0f), 0.0f, 1e-6f,
        "calibrated white maps to zero");
    expect_near(qti_normalize(1100.0f, 100.0f, 1100.0f), 1.0f, 1e-6f,
        "calibrated black maps to one");
    expect_near(qti_normalize(600.0f, 100.0f, 1100.0f), 0.5f, 1e-6f,
        "calibration midpoint maps near half");
    expect_near(qti_normalize(50.0f, 100.0f, 1100.0f), 0.0f, 1e-6f,
        "below white clamps to zero");
    expect_near(qti_normalize(1500.0f, 100.0f, 1100.0f), 1.0f, 1e-6f,
        "above black clamps to one");
}

static int prop_normalize_q1000(int raw, int white_time, int black_time) {
    int denom = black_time - white_time;
    int obs;
    if (denom == 0) {
        return raw >= black_time ? 1000 : 0;
    }
    obs = ((raw - white_time) * 1000) / denom;
    if (obs < 0) return 0;
    if (obs > 1000) return 1000;
    return obs;
}

static void test_propeller_qti_normalization_parity(void) {
    const int white = 40;
    const int black = 350;
    const int raws[] = {0, 39, 40, 45, 195, 230, 265, 328, 350, 700};
    for (int i = 0; i < (int)(sizeof(raws) / sizeof(raws[0])); i++) {
        float sim = qti_normalize((float)raws[i], (float)white, (float)black);
        float prop = (float)prop_normalize_q1000(raws[i], white, black) / 1000.0f;
        expect_near(prop, sim, 0.0011f,
            "Propeller q1000 normalization matches sim normalization");
    }
}

static void test_continuous_actions(void) {
    float actions[2] = {-2.0f, 2.0f};
    LineFollowWheelCommand cmd = map_actions(actions, 0.5f, 0.05f, 1.0f, 1.0f);
    expect_near(actions[0], -1.0f, 1e-6f, "left action clamps low");
    expect_near(actions[1], 1.0f, 1e-6f, "right action clamps high");
    expect_near(cmd.left_action, -1.0f, 1e-6f, "clamped left command keeps signed action for penalties");
    expect_near(cmd.left_mps, 0.0f, 1e-6f, "negative left command clamps to stopped wheel speed");
    expect_near(cmd.right_mps, 0.5f, 1e-6f, "clamped right action maps to positive wheel speed");

    float tiny[2] = {0.02f, -0.04f};
    cmd = map_actions(tiny, 0.5f, 0.05f, 1.0f, 1.0f);
    expect_near(cmd.left_mps, 0.0f, 1e-6f, "left deadband maps tiny command to zero");
    expect_near(cmd.right_mps, 0.0f, 1e-6f, "right deadband maps tiny command to zero");

    float polarity[2] = {1.0f, -1.0f};
    cmd = map_actions(polarity, 0.35f, 0.0f, 1.0f, 1.0f);
    expect_near(cmd.left_mps, 0.35f, 1e-6f, "positive left command drives left wheel forward");
    expect_near(cmd.right_action, -1.0f, 1e-6f, "negative right command keeps signed action for penalties");
    expect_near(cmd.right_mps, 0.0f, 1e-6f, "negative right command clamps to stopped wheel speed");
}

static void test_measured_geometry_defaults(void) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);

    expect_near(env.dt, 0.065f, 1e-6f, "nominal control period matches tiny feed-forward measurement");
    expect_near(env.dt_min, 0.045f, 1e-6f, "minimum randomized control period covers tiny feed-forward measurement");
    expect_near(env.dt_max, 0.090f, 1e-6f, "maximum randomized control period covers tiny feed-forward measurement");
    expect_near(env.max_wheel_speed_mps * env.dt, 0.00065f, 1e-6f,
        "full-speed travel per decision is reduced for robot smoke tests");
    expect_true(env.model_dt_enabled == 1, "model-aware dt scaling is enabled by default");
    expect_true(env.policy_hidden_size == 8, "default policy timing hidden size matches config default");
    expect_true(env.policy_num_layers == 0, "default policy timing recurrent layer count matches config default");
    expect_near(env.progress_reward_scale, 2.0f, 1e-6f,
        "dense progress reward is weighted above per-tick costs");
    expect_near(env.centerline_penalty_scale, 0.0f, 1e-6f,
        "centerline accuracy is scored at progress checkpoints");
    expect_near(env.centerline_reward_interval_m, 0.025f, 1e-6f,
        "centerline bonus pays out every twenty-five millimeters");
    expect_near(env.centerline_reward_scale, 1.0f, 1e-6f,
        "centerline milestone bonus has default scale");
    expect_near(env.success_reward, 1.0f, 1e-6f,
        "survival success bonus has default scale");
    expect_near(env.turn_penalty_scale, 0.0025f, 1e-6f,
        "turn command penalty is tiny but always present");
    expect_near(env.time_penalty, 0.02f, 1e-6f,
        "per-tick time penalty discourages crawling or stopping");
    expect_near(env.idle_penalty, 0.02f, 1e-6f,
        "idle wheel penalty prevents centered stop reward hacking");
    expect_near(env.min_wheel_action, 0.08f, 1e-6f,
        "idle threshold requires at least one wheel command above a small action");
    expect_near(env.track_complete_margin_m, 0.02f, 1e-6f,
        "completion margin is scaled for paper-sized tracks");
    expect_near(env.wheel_base_m, 0.125f, 1e-6f, "measured tire-to-tire width is default wheel base");
    expect_near(env.max_wheel_speed_mps, 0.010f, 1e-6f, "robot max wheel speed matches slow control loop");
    expect_near(env.body_ahead_m, 0.040f, 1e-6f, "measured body ahead of axle is default");
    expect_near(env.body_behind_m, 0.090f, 1e-6f, "measured body behind axle is default");
    expect_near(env.tire_diameter_m, 0.065f, 1e-6f, "measured tire diameter is default");
    expect_near(env.sensor_forward_m, 0.045f, 1e-6f, "measured sensor forward offset is default");
    expect_near(env.sensor_inner_lateral_m, 0.020f, 1e-6f,
        "measured inner sensor lateral offset is default");
    expect_near(env.sensor_outer_lateral_m, 0.040f, 1e-6f,
        "measured outer sensor lateral offset is default");
    expect_near(env.sensor_forward_jitter_m, 0.0025f, 1e-6f,
        "sensor forward jitter default is two and a half millimeters");
    expect_near(env.sensor_lateral_jitter_m, 0.0025f, 1e-6f,
        "sensor lateral jitter default is two and a half millimeters");
    expect_near(env.qti_white_time, 40.0f, 1e-6f,
        "nominal QTI white calibration follows real telemetry");
    expect_near(env.qti_black_time, 350.0f, 1e-6f,
        "nominal QTI black calibration allows full-dark readings");
    expect_near(env.qti_white_jitter, 25.0f, 1e-6f,
        "QTI white response is randomized for training");
    expect_near(env.qti_black_jitter, 120.0f, 1e-6f,
        "QTI black response is randomized for training");
    expect_near(env.line_width_jitter_m, 0.004f, 1e-6f,
        "line width is randomized for training");
    expect_near(env.line_edge_softness_jitter_m, 0.004f, 1e-6f,
        "line edge softness is randomized for training");
    expect_near(env.start_lateral_offset_m, 0.050f, 1e-6f,
        "start offset range reaches outer sensors");

    expect_near(env.sensor_forward[0], 0.045f, 1e-6f, "outer-left sensor forward default");
    expect_near(env.sensor_lateral[0], 0.040f, 1e-6f, "outer-left sensor lateral default");
    expect_near(env.sensor_forward[1], 0.045f, 1e-6f, "inner-left sensor forward default");
    expect_near(env.sensor_lateral[1], 0.020f, 1e-6f, "inner-left sensor lateral default");
    expect_near(env.sensor_forward[2], 0.045f, 1e-6f, "inner-right sensor forward default");
    expect_near(env.sensor_lateral[2], -0.020f, 1e-6f, "inner-right sensor lateral default");
    expect_near(env.sensor_forward[3], 0.045f, 1e-6f, "outer-right sensor forward default");
    expect_near(env.sensor_lateral[3], -0.040f, 1e-6f, "outer-right sensor lateral default");
}

static void test_episode_dt_randomization(void) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.rng = 123u;

    bool varied = false;
    float first = -1.0f;
    for (int i = 0; i < 64; i++) {
        randomize_episode_params(&env);
        expect_true(env.episode_dt >= 0.045f && env.episode_dt <= 0.090f,
            "episode dt samples inside configured range");
        if (i == 0) {
            first = env.episode_dt;
        } else if (fabsf(env.episode_dt - first) > 1e-5f) {
            varied = true;
        }
    }
    expect_true(varied, "episode dt is randomized across resets");

    env.dt = 0.042f;
    env.dt_min = 0.0f;
    env.dt_max = 0.0f;
    randomize_episode_params(&env);
    expect_near(env.episode_dt, 0.042f, 1e-6f,
        "episode dt falls back to nominal dt when range is disabled");

    env.dt_min = 0.090f;
    env.dt_max = 0.030f;
    randomize_episode_params(&env);
    expect_true(env.episode_dt >= 0.030f && env.episode_dt <= 0.090f,
        "episode dt handles reversed min/max safely");
}

static void test_model_timing_penalizes_larger_policy(void) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    apply_model_timing(&env);
    expect_near(env.dt, 0.065f, 1e-6f, "H8 feed-forward timing matches measured drive loop");
    expect_near(env.dt_min, 0.045f, 1e-6f, "H8 feed-forward timing preserves min ratio");
    expect_near(env.dt_max, 0.090f, 1e-6f, "H8 feed-forward timing preserves max ratio");
    expect_true(env.max_steps == 800, "H8 feed-forward keeps default real-time horizon");
    expect_true(env.lost_line_limit == 16, "H8 feed-forward keeps default lost-line grace");

    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.policy_hidden_size = 8;
    env.policy_num_layers = 1;
    apply_model_timing(&env);
    expect_near(env.dt, 0.188f, 1e-6f, "H8 recurrent timing uses measured slower loop");
    expect_near(env.dt_min, 0.130154f, 1e-5f, "H8 recurrent timing scales min dt");
    expect_near(env.dt_max, 0.260308f, 1e-5f, "H8 recurrent timing scales max dt");
    expect_true(env.max_steps == 277, "H8 recurrent reduces max steps to preserve wall-clock horizon");
    expect_true(env.lost_line_limit == 6, "H8 recurrent reduces lost-line grace to preserve wall-clock time");

    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.policy_hidden_size = 4;
    env.policy_num_layers = 0;
    apply_model_timing(&env);
    expect_near(env.dt, 0.055f, 1e-6f, "H4 feed-forward timing is faster than H8 feed-forward");
    expect_true(env.max_steps == 946, "H4 feed-forward gets more update steps for the same wall-clock horizon");
}

static void test_start_lateral_offset_samples_sensor_targets(void) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.rng = 123u;
    env.sensor_lateral_jitter_m = 0.0f;
    env.episode_line_width_m = 0.0f;
    sensor_layout(&env);

    bool seen[LINE_FOLLOW_OBS_SIZE + 1] = {false};
    const float targets[LINE_FOLLOW_OBS_SIZE + 1] = {
        -env.sensor_lateral[0],
        -env.sensor_lateral[1],
        -env.sensor_lateral[2],
        -env.sensor_lateral[3],
        0.0f,
    };

    for (int i = 0; i < 256; i++) {
        float offset = sample_start_lateral_offset(&env);
        for (int j = 0; j < LINE_FOLLOW_OBS_SIZE + 1; j++) {
            if (fabsf(offset - targets[j]) < 1e-6f) {
                seen[j] = true;
            }
        }
    }

    expect_true(seen[0], "start sampler can place outer-left sensor on line");
    expect_true(seen[1], "start sampler can place inner-left sensor on line");
    expect_true(seen[2], "start sampler can place inner-right sensor on line");
    expect_true(seen[3], "start sampler can place outer-right sensor on line");
    expect_true(seen[4], "start sampler can place robot centered on line");
}

static void test_track_generation(void) {
    LineFollowTrack track;
    memset(&track, 0, sizeof(track));
    expect_true(generate_straight(&track, 1.0f, 0.006f, 1.0f),
        "straight track generation succeeds");
    expect_true(track.sample_count > 2, "straight track has samples");
    expect_true(track.sample_count <= LINE_FOLLOW_MAX_TRACK_SAMPLES, "straight track respects sample cap");

    for (int i = 0; i < track.sample_count; i++) {
        expect_true(fabsf(track.samples[i].x) <= 1.0f, "straight sample x stays in bounds");
        expect_true(fabsf(track.samples[i].y) <= 1.0f, "straight sample y stays in bounds");
        if (i > 0) {
            expect_true(track.samples[i].progress_s >= track.samples[i - 1].progress_s,
                "straight track progress is monotonic");
        }
    }

    LineFollowNearest on_line = nearest_track(&track, 0.0f, 0.0f);
    expect_near(on_line.distance_m, 0.0f, 1e-5f, "nearest distance is zero on centerline");
    LineFollowNearest off_line = nearest_track(&track, 0.0f, 0.05f);
    expect_near(off_line.distance_m, 0.05f, 1e-4f, "nearest distance matches off-line offset");
    expect_true(off_line.signed_lateral_m > 0.0f, "positive y is positive signed lateral on theta-zero straight");

    unsigned int rng = 123u;
    expect_true(generate_track(&track, &rng, LINE_FOLLOW_TRACK_RANDOM, 0.006f, 1.0f),
        "capped randomized track generation succeeds");
    expect_true(track.sample_count <= LINE_FOLLOW_MAX_TRACK_SAMPLES, "randomized generation respects sample cap");
}

static void test_episode_progress_starts_at_zero(void) {
    float obs[4] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    int families[] = {
        LINE_FOLLOW_TRACK_STRAIGHT,
        LINE_FOLLOW_TRACK_ARC,
        LINE_FOLLOW_TRACK_S_CURVE,
        LINE_FOLLOW_TRACK_OVAL,
        LINE_FOLLOW_TRACK_RANDOM,
    };

    for (int family_idx = 0; family_idx < 5; family_idx++) {
        env.track_family = families[family_idx];
        for (int episode = 0; episode < 64; episode++) {
            c_reset(&env);
            expect_near(env.episode_progress, 0.0f, 1e-6f,
                "episode progress starts from zero after reset");
            memset(&env.log, 0, sizeof(env.log));
            add_log(&env);
            expect_near(env.log.perf, 0.0f, 1e-6f,
                "perf does not include absolute nearest-track progress at reset");
        }
    }
}

static void test_perf_penalizes_centerline_error(void) {
    float obs[4] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    expect_true(generate_straight(&env.track, 1.0f, env.line_width_m, env.track_bounds_m),
        "perf test track generation succeeds");
    env.episode_line_width_m = env.line_width_m;

    env.tick = 10;
    env.episode_progress = perf_target_progress_m(&env);
    int target_checkpoints = perf_target_checkpoints(&env);
    env.perf_quality_sum = (float)target_checkpoints;
    env.perf_checkpoint_count = target_checkpoints;
    env.centerline_error = 0.0f;
    env.centerline_error_sum = 0.0f;
    add_log(&env);
    expect_near(env.log.perf, 1.0f, 1e-6f,
        "all checkpoint qualities at one gives full perf");
    expect_near(env.log.score, 1.0f, 1e-6f,
        "score is normalized to full perf");
    expect_near(env.log.progress_frac, 1.0f, 1e-6f,
        "all target checkpoints reached logs full progress fraction");
    expect_near(env.log.distance_progress_frac, 1.0f, 1e-6f,
        "full target distance logs full distance progress fraction");
    expect_near(env.log.checkpoint_accuracy, 1.0f, 1e-6f,
        "all checkpoint qualities at one logs full checkpoint accuracy");
    expect_near(env.log.checkpoints, (float)target_checkpoints, 1e-6f,
        "checkpoint count is logged");
    expect_near(env.log.target_checkpoints, (float)target_checkpoints, 1e-6f,
        "target checkpoint count is logged");
    expect_near(env.log.progress_m, env.episode_progress, 1e-6f,
        "raw progress meters are still logged");

    memset(&env.log, 0, sizeof(env.log));
    env.tick = 10;
    env.episode_progress = perf_target_progress_m(&env);
    env.perf_quality_sum = 0.0f;
    env.perf_checkpoint_count = target_checkpoints;
    env.centerline_error = env.sensor_outer_lateral_m;
    env.centerline_error_sum = env.sensor_outer_lateral_m * 10.0f;
    add_log(&env);
    expect_near(env.log.perf, 0.0f, 1e-6f,
        "zero checkpoint qualities gives zero perf even with full progress");
}

static void test_perf_checkpoint_quality(void) {
    float obs[4] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    env.episode_line_width_m = env.line_width_m;

    expect_near(accuracy_score(&env, 0.000f), 1.0f, 1e-6f,
        "zero millimeter centerline error has full accuracy");
    expect_near(accuracy_score(&env, 0.005f), 1.0f, 1e-6f,
        "five millimeter centerline error has full accuracy");
    expect_near(accuracy_score(&env, 0.010f), 0.9f, 1e-6f,
        "ten millimeter centerline error has ninety percent accuracy");
    expect_near(accuracy_score(&env, 0.015f), 0.8f, 1e-6f,
        "fifteen millimeter centerline error has eighty percent accuracy");
    expect_near(accuracy_score(&env, 0.020f), 0.7f, 1e-6f,
        "twenty millimeter centerline error has seventy percent accuracy");

    float first = centerline_progress_reward(&env, 0.025f, 0.0f);
    expect_true(first > 0.0f, "first checkpoint can pay reward");
    expect_near(env.perf_quality_sum, 1.0f, 1e-6f,
        "centered checkpoint adds full perf quality");
    expect_true(env.perf_checkpoint_count == 1,
        "centered checkpoint increments perf checkpoint count");

    centerline_progress_reward(&env, 0.025f, 0.020f);
    expect_near(env.perf_quality_sum, 1.7f, 1e-6f,
        "twenty millimeter checkpoint adds seventy percent perf quality");
    expect_true(env.perf_checkpoint_count == 2,
        "second checkpoint increments perf checkpoint count");
}

static void test_progress_rejects_discontinuous_nearest_jump(void) {
    float obs[4] = {0};
    float actions[2] = {1.0f, 1.0f};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    env.track_family = LINE_FOLLOW_TRACK_OVAL;

    for (int episode = 0; episode < 64; episode++) {
        c_reset(&env);
        c_step(&env);
        expect_true(env.episode_progress <= 0.006f,
            "one physical step cannot receive a near-complete oval progress jump");
    }
}

static void test_centerline_milestone_and_motion_rewards(void) {
    float obs[4] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);

    float first = centerline_progress_reward(&env, 0.024f, 0.0f);
    expect_near(first, 0.0f, 1e-6f,
        "centerline bonus does not pay before twenty-five millimeters");
    float second = centerline_progress_reward(&env, 0.001f, 0.0f);
    expect_near(second, 1.0f, 1e-6f,
        "centerline bonus pays when cumulative progress reaches twenty-five millimeters");
    expect_near(env.centerline_reward_progress_m, 0.0f, 1e-6f,
        "centerline bonus accumulator subtracts paid interval");

    float near_bonus = centerline_progress_reward(&env, 0.025f, 0.0f);
    float far_bonus = centerline_progress_reward(&env, 0.025f, env.sensor_outer_lateral_m);
    expect_true(near_bonus > far_bonus,
        "centerline milestone reward is higher near the centerline");

    float stopped = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 0.0f, true, 0.0f);
    expect_near(stopped, -env.time_penalty - env.idle_penalty, 1e-6f,
        "centered stopped robot gets time and idle penalties");

    float straight = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 0.0f, false, 0.0f);
    float turning = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 1.0f, false, 0.0f);
    expect_true(straight > turning,
        "turn command penalty applies even without action changes");

    float fast_centered = compute_reward(&env, 0.025f, 0.0f, 0.0f, true, 0.1f,
        0.0f, 0.0f, 0.0f, false, 0.0f);
    float fast_off_center = compute_reward(&env, 0.025f, 0.020f, 0.0f, true, 0.1f,
        0.0f, 0.0f, 0.0f, false, 0.0f);
    expect_true(fast_centered > fast_off_center,
        "dense progress reward is scaled by accuracy");
}

static void test_sensor_and_reward_behavior(void) {
    float obs[4] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    expect_true(generate_straight(&env.track, 1.0f, env.line_width_m, env.track_bounds_m),
        "test straight track generation succeeds");
    reset_runtime_pose(&env, 0.0f, 0.0f, 0.0f);
    compute_observations(&env);
    env.centerline_error = 0.0f;

    expect_near(obs[0], obs[3], 1e-5f, "centered outer QTI readings are symmetric");
    expect_near(obs[1], obs[2], 1e-5f, "centered inner QTI readings are symmetric");
    expect_true(obs[0] <= 0.01f && obs[1] <= 0.01f && obs[2] <= 0.01f && obs[3] <= 0.01f,
        "centered measured sensors straddle the narrow default line");
    expect_true(line_visible_to_sensor_array(&env),
        "centered line between the sensors is still visible to the array");

    reset_runtime_pose(&env, 0.0f, 0.02f, 0.0f);
    compute_observations(&env);
    env.centerline_error = 0.02f;
    float left_darkness = obs[0] + obs[1];
    float right_darkness = obs[2] + obs[3];
    expect_true(right_darkness > left_darkness, "left-shifted robot makes right QTI readings darker");

    reset_runtime_pose(&env, 0.0f, -0.02f, 0.0f);
    compute_observations(&env);
    left_darkness = obs[0] + obs[1];
    right_darkness = obs[2] + obs[3];
    expect_true(left_darkness > right_darkness, "right-shifted robot makes left QTI readings darker");

    reset_runtime_pose(&env, 0.0f, 0.09f, 0.0f);
    compute_observations(&env);
    expect_true(!line_visible_to_sensor_array(&env),
        "line fully right of the far right sensor is treated as lost");

    for (int i = 0; i < 4; i++) {
        env.observations[i] = 1.0f;
    }
    expect_true(!line_visible_to_sensor_array(&env),
        "lost-line terminal geometry does not depend on sensor observations");

    reset_runtime_pose(&env, 0.0f, -0.09f, 0.0f);
    compute_observations(&env);
    expect_true(!line_visible_to_sensor_array(&env),
        "line fully left of the far left sensor is treated as lost");

    for (int i = 0; i < 4; i++) {
        obs[i] = 0.5f;
    }
    float near_bonus = accuracy_score(&env, 0.0f);
    float far_bonus = accuracy_score(&env, 0.04f);
    float near_reward = compute_reward(&env, 0.025f, 0.0f, 0.0f, true, 0.1f,
        0.0f, near_bonus, 0.0f, false, 0.0f);
    float far_reward = compute_reward(&env, 0.025f, 0.04f, 0.0f, true, 0.1f,
        0.0f, far_bonus, 0.0f, false, 0.0f);
    expect_true(near_reward > far_reward,
        "reward changes with privileged centerline distance even when observations are ambiguous");
}

static void test_negative_wheel_command_is_penalized_without_terminal(void) {
    float obs[4] = {0};
    float actions[2] = {-0.5f, 0.5f};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    expect_true(generate_straight(&env.track, 1.0f, env.line_width_m, env.track_bounds_m),
        "negative command test straight track generation succeeds");
    env.episode_line_width_m = env.line_width_m;
    env.episode_line_edge_softness_m = env.line_edge_softness_m;
    env.lost_line_limit = 100;
    reset_runtime_pose(&env, env.track.samples[0].x, env.track.samples[0].y, 0.0f);
    compute_observations(&env);

    c_step(&env);

    expect_near(env.v_left_cmd, 0.0f, 1e-6f,
        "negative left command clamps to stopped wheel in sim");
    expect_true(env.v_right_cmd > 0.0f,
        "positive right command still drives the right wheel");
    expect_near(terminals[0], 0.0f, 1e-6f,
        "negative wheel command is no longer an immediate terminal");
    expect_near(env.log.n, 0.0f, 1e-6f,
        "negative wheel command does not log a terminal episode by itself");
    expect_true(env.negative_action_steps == 1,
        "negative wheel command is counted for episode diagnostics");
    expect_true(rewards[0] < 0.0f,
        "negative wheel command receives normal reward penalties");
}

static void reset_centered_straight_episode(LineFollow* env) {
    expect_true(generate_straight(&env->track, 1.0f, env->line_width_m, env->track_bounds_m),
        "terminal reward test straight track generation succeeds");
    env->episode_line_width_m = env->line_width_m;
    env->episode_line_edge_softness_m = env->line_edge_softness_m;
    env->tick = 0;
    env->lost_line_steps = 0;
    env->idle_steps = 0;
    env->episode_return = 0.0f;
    env->episode_progress = 0.0f;
    env->last_progress = 0.0f;
    env->centerline_reward_progress_m = 0.0f;
    env->perf_quality_sum = 0.0f;
    env->perf_checkpoint_count = 0;
    env->forward_speed_sum = 0.0f;
    env->speed_frac_sum = 0.0f;
    env->centerline_error_sum = 0.0f;
    clear_terminal_info(env);
    reset_runtime_pose(env, env->track.samples[0].x, env->track.samples[0].y, 0.0f);
    compute_observations(env);
}

static void test_timeout_success_reward_requires_progress(void) {
    float obs[4] = {0};
    float actions[2] = {0.25f, 0.25f};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    env.max_steps = 1;
    env.lost_line_limit = 100;
    reset_centered_straight_episode(&env);

    c_step(&env);

    expect_near(terminals[0], 1.0f, 1e-6f,
        "timeout emits terminal flag");
    expect_near(env.log.terminal_success, 0.0f, 1e-6f,
        "timeout without a progress checkpoint earns no success bonus");
    expect_true(rewards[0] < 0.1f,
        "timeout reward is not dominated by survival without progress");

    actions[0] = 1.0f;
    actions[1] = 1.0f;
    rewards[0] = 0.0f;
    terminals[0] = 0.0f;
    memset(&env.log, 0, sizeof(env.log));
    env.max_steps = 1;
    env.lost_line_limit = 100;
    env.centerline_reward_interval_m = 0.00005f;
    reset_centered_straight_episode(&env);

    c_step(&env);

    float moving_return = env.log.episode_return;
    expect_near(terminals[0], 1.0f, 1e-6f,
        "moving timeout emits terminal flag");
    expect_true(env.log.perf > 0.0f,
        "moving timeout records checkpoint progress");
    expect_true(env.log.terminal_success > 0.0f,
        "moving timeout success bonus scales from progress perf");
    expect_true(moving_return > 0.0f,
        "moving timeout can earn positive progress-based return");

    actions[0] = 0.0f;
    actions[1] = 0.0f;
    rewards[0] = 0.0f;
    terminals[0] = 0.0f;
    memset(&env.log, 0, sizeof(env.log));
    env.max_steps = 1;
    env.lost_line_limit = 100;
    env.centerline_reward_interval_m = 0.00005f;
    reset_centered_straight_episode(&env);

    c_step(&env);

    expect_near(terminals[0], 1.0f, 1e-6f,
        "idle timeout still emits terminal flag");
    expect_true(rewards[0] < 0.1f,
        "idle timeout does not earn success reward");
    expect_true(env.log.episode_return < moving_return,
        "logged idle timeout return is worse than progress timeout");
}

static void test_randomized_sensor_response_clamps_observations(void) {
    float obs[4] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);

    env.qti_white_jitter = 200.0f;
    env.qti_black_jitter = 3000.0f;
    env.sensor_noise_std = 2000.0f;
    env.line_width_jitter_m = 0.010f;
    env.line_edge_softness_jitter_m = 0.010f;

    for (int episode = 0; episode < 32; episode++) {
        c_reset(&env);
        for (int i = 0; i < 4; i++) {
            expect_true(obs[i] >= 0.0f && obs[i] <= 1.0f,
                "randomized reset observations stay clamped to [0, 1]");
            expect_true(env.sensor_black_time[i] > env.sensor_white_time[i],
                "randomized QTI black calibration stays above white calibration");
        }
        expect_true(env.episode_line_width_m >= 0.004f,
            "randomized line width stays positive");
        expect_true(env.episode_line_edge_softness_m >= 0.001f,
            "randomized line edge softness stays positive");
        actions[0] = rand_signed(&env.rng);
        actions[1] = rand_signed(&env.rng);
        c_step(&env);
        for (int i = 0; i < 4; i++) {
            expect_true(obs[i] >= 0.0f && obs[i] <= 1.0f,
                "randomized step observations stay clamped to [0, 1]");
        }
    }
}

int main(void) {
    test_qti_normalization();
    test_propeller_qti_normalization_parity();
    test_continuous_actions();
    test_measured_geometry_defaults();
    test_episode_dt_randomization();
    test_model_timing_penalizes_larger_policy();
    test_start_lateral_offset_samples_sensor_targets();
    test_track_generation();
    test_episode_progress_starts_at_zero();
    test_perf_penalizes_centerline_error();
    test_perf_checkpoint_quality();
    test_progress_rejects_discontinuous_nearest_jump();
    test_centerline_milestone_and_motion_rewards();
    test_sensor_and_reward_behavior();
    test_negative_wheel_command_is_penalized_without_terminal();
    test_timeout_success_reward_requires_progress();
    test_randomized_sensor_response_clamps_observations();

    if (failures > 0) {
        fprintf(stderr, "%d line_follow test failures\n", failures);
        return 1;
    }

    printf("line_follow tests passed\n");
    return 0;
}
