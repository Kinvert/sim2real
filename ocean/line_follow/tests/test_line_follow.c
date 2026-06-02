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

static float total_track_turn(const LineFollowTrack* track) {
    float total = 0.0f;
    if (track->sample_count < 3) {
        return 0.0f;
    }

    float prev = atan2f(track->samples[0].tangent_y, track->samples[0].tangent_x);
    for (int i = 1; i < track->sample_count; i++) {
        float heading = atan2f(track->samples[i].tangent_y, track->samples[i].tangent_x);
        total += angle_diff(heading, prev);
        prev = heading;
    }
    return total;
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

static void test_rawdiff_observation_mapping(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    env.qti_white_time = 80.0f;
    env.qti_black_time = 350.0f;
    env.obs_raw_diff = 1;
    env.obs_raw_diff_gain = 3.0f;
    env.obs_raw_diff_span = 0.0f;
    env.sensor_noise_std = 0.0f;

    expect_true(generate_straight(&env.track, 1.0f, env.line_width_m, env.track_bounds_m),
        "raw-diff test straight track generation succeeds");

    reset_runtime_pose(&env, 0.0f, 0.09f, 0.0f);
    compute_observations(&env);
    expect_true(obs[0] <= 0.001f && obs[1] <= 0.001f && obs[2] <= 0.001f,
        "raw-diff all-white observation stays all white");

    reset_runtime_pose(&env, 0.0f, 0.02f, 0.0f);
    compute_observations(&env);
    expect_true(obs[2] > obs[0] && obs[2] > obs[1],
        "raw-diff preserves the darker side sensor direction");
    expect_true(obs[2] >= 0.5f,
        "raw-diff gain can lift side-edge contrast over the black threshold");
}

static void test_continuous_actions(void) {
    float actions[2] = {-2.0f, 2.0f};
    LineFollowWheelCommand cmd = map_actions(actions, 0.5f, 0.05f,
        0.065f, 0.0f, 1.0f, 1.0f);
    expect_near(actions[0], -1.0f, 1e-6f, "left action clamps low");
    expect_near(actions[1], 1.0f, 1e-6f, "right action clamps high");
    expect_near(cmd.left_action, -1.0f, 1e-6f, "clamped left command keeps signed normalized action");
    expect_near(cmd.left_mps, 0.0f, 1e-6f, "minus-one left command maps to stopped wheel speed");
    expect_near(cmd.right_mps, 0.5f, 1e-6f, "plus-one right action maps to max wheel speed");

    float tiny[2] = {0.02f, -0.04f};
    cmd = map_actions(tiny, 0.5f, 0.05f, 0.065f, 0.0f, 1.0f, 1.0f);
    expect_near(cmd.left_mps, 0.0f, 1e-6f, "near-zero left action maps to stopped wheel");
    expect_near(cmd.right_mps, 0.0f, 1e-6f, "negative near-zero right action maps to stopped wheel");

    float stop[2] = {-0.96f, -0.88f};
    cmd = map_actions(stop, 0.5f, 0.05f, 0.065f, 0.0f, 1.0f, 1.0f);
    expect_near(cmd.left_mps, 0.0f, 1e-6f, "left negative command maps to zero");
    expect_near(cmd.right_mps, 0.0f, 1e-6f,
        "right negative command maps to zero");

    float polarity[2] = {1.0f, -1.0f};
    cmd = map_actions(polarity, 0.35f, 0.0f, 0.065f, 0.0f, 1.0f, 1.0f);
    expect_near(cmd.left_mps, 0.35f, 1e-6f, "positive left command drives left wheel forward");
    expect_near(cmd.right_action, -1.0f, 1e-6f, "minus-one right command keeps signed normalized action");
    expect_near(cmd.right_mps, 0.0f, 1e-6f, "minus-one right command maps to stopped wheel speed");
    expect_near(raw_action_from_command(0.0f), 0.0f, 1e-6f,
        "zero wheel command maps to zero policy action");
    expect_near(raw_action_from_command(0.5f), 0.5f, 1e-6f,
        "half wheel command maps to half policy action");
    expect_near(raw_action_from_command(1.0f), 1.0f, 1e-6f,
        "full wheel command maps to plus-one policy action");

    float min_mps = wheel_ticks_per_sec_to_mps(6.0f, 0.065f);
    float floored[2] = {0.0f, -1.0f};
    cmd = map_actions(floored, 0.5f, 0.05f, 0.065f, 6.0f, 1.0f, 1.0f);
    expect_near(min_mps, 0.019144f, 1e-6f,
        "six ticks per second converts to measured anti-stall mps");
    expect_near(cmd.left_mps, min_mps, 1e-6f,
        "zero left policy command is floored to anti-stall speed");
    expect_near(cmd.right_mps, min_mps, 1e-6f,
        "negative right policy command is floored to anti-stall speed");

    float capped[2] = {0.0f, 1.0f};
    cmd = map_actions(capped, 0.005f, 0.05f, 0.065f, 6.0f, 1.0f, 1.0f);
    expect_near(cmd.left_mps, 0.005f, 1e-6f,
        "anti-stall floor caps at configured max wheel speed");
    expect_near(cmd.right_mps, 0.005f, 1e-6f,
        "anti-stall floor does not exceed max wheel speed");
}

static void test_measured_geometry_defaults(void) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);

    expect_near(env.dt, 0.0065f, 1e-6f, "nominal control period matches measured fixed-point Prop C loop timing");
    expect_near(env.dt_min, 0.004f, 1e-6f, "minimum randomized control period follows measured body time");
    expect_near(env.dt_max, 0.009f, 1e-6f, "maximum randomized control period covers measured real-loop jitter");
    expect_near(env.max_wheel_speed_mps * env.dt, 0.000754f, 1e-6f,
        "full-speed travel per decision matches the fixed-point deployment window");
    expect_true(env.model_dt_enabled == 1, "model-aware dt scaling is enabled by default");
    expect_true(env.policy_hidden_size == 8, "default policy timing hidden size matches config default");
    expect_true(env.policy_num_layers == 0, "default policy timing feed-forward layer count matches config default");
    expect_near(env.progress_reward_scale, 1.0f, 1e-6f,
        "progress reward is normalized to full-speed tick progress");
    expect_near(env.path_efficiency_perf_weight, 0.0f, 1e-6f,
        "path-efficiency perf weighting is opt-in from training config");
    expect_near(env.wasted_motion_penalty_scale, 0.0f, 1e-6f,
        "wasted-motion penalty is opt-in from training config");
    expect_near(env.centerline_penalty_scale, 0.0f, 1e-6f,
        "separate dense centerline penalty is disabled because progress is accuracy-scaled");
    expect_near(env.centerline_reward_interval_m, 0.010f, 1e-6f,
        "centerline bonus pays out every ten millimeters");
    expect_near(env.success_reward, 1.0f, 1e-6f,
        "survival success bonus has default scale");
    expect_near(env.heading_penalty_scale, 0.0f, 1e-6f,
        "separate heading penalty is disabled because checkpoint accuracy handles it indirectly");
    expect_near(env.lost_line_penalty, 0.03f, 1e-6f,
        "lost line penalty preserves shaping without dominating checkpoint accuracy");
    expect_near(env.lost_line_recovery_scale, 0.0f, 1e-6f,
        "lost-line recovery shaping is opt-in from training config");
    expect_near(env.off_track_terminal_penalty, 1.0f, 1e-6f,
        "off-track terminal penalty defaults to the largest clipped negative reward");
    expect_near(env.action_bound_penalty_scale, 0.01f, 1e-6f,
        "raw action bound penalty stays small so it does not suppress usable turn commands");
    expect_near(env.turn_penalty_scale, 0.0f, 1e-6f,
        "turn command penalty is disabled so steering itself is not punished");
    expect_near(env.ambiguous_turn_penalty_scale, 0.0f, 1e-6f,
        "ambiguous-observation turn penalty is opt-in from training config");
    expect_near(env.steering_correction_scale, 0.30f, 1e-6f,
        "privileged steering correction reward teaches the signed turn direction");
    expect_near(env.turn_speed_penalty_scale, 0.05f, 1e-6f,
        "turn speed penalty discourages weak outside-wheel turns");
    expect_near(env.min_turn_outer_action, 0.65f, 1e-6f,
        "corrective turns target a fast outside wheel");
    expect_near(env.time_penalty, 0.005f, 1e-6f,
        "per-tick time penalty discourages crawling or stopping");
    expect_near(env.idle_penalty, 0.02f, 1e-6f,
        "idle wheel penalty prevents centered stop reward hacking");
    expect_near(env.min_wheel_action, 0.35f, 1e-6f,
        "idle threshold pushes pivot turns to keep the outside wheel moving");
    expect_near(env.min_drive_ticks_per_sec, 6.0f, 1e-6f,
        "anti-stall wheel floor defaults to the first smooth measured slow speed");
    expect_near(env.avg_speed_perf_target_mps, 0.18f, 1e-6f,
        "average-speed perf target rewards sane pace without forcing top speed");
    expect_near(env.track_complete_margin_m, 0.02f, 1e-6f,
        "completion margin is scaled for paper-sized tracks");
    expect_near(env.wheel_base_m, 0.125f, 1e-6f, "measured tire-to-tire width is default wheel base");
    expect_near(env.max_wheel_speed_mps, 0.116f, 1e-6f,
        "robot max wheel speed uses the doubled deployment window");
    expect_near(env.body_ahead_m, 0.040f, 1e-6f, "measured body ahead of axle is default");
    expect_near(env.body_behind_m, 0.090f, 1e-6f, "measured body behind axle is default");
    expect_near(env.tire_diameter_m, 0.065f, 1e-6f, "measured tire diameter is default");
    expect_near(env.sensor_forward_m, 0.0435f, 1e-6f,
        "measured twisted sensor forward offset is default");
    expect_near(env.sensor_side_lateral_m, 0.018f, 1e-6f,
        "measured twisted side sensor lateral offset is default");
    expect_near(env.sensor_lateral_center_m, 0.0f, 1e-6f,
        "default three-sensor layout remains centered on the robot");
    expect_near(env.sensor_forward_jitter_m, 0.003f, 1e-6f,
        "sensor forward jitter default covers small mounting errors");
    expect_near(env.sensor_lateral_jitter_m, 0.004f, 1e-6f,
        "sensor lateral jitter default covers hand-built mounting spread");
    expect_near(env.qti_white_time, 40.0f, 1e-6f,
        "nominal QTI white calibration follows real telemetry");
    expect_near(env.qti_black_time, 350.0f, 1e-6f,
        "nominal QTI black calibration allows full-dark readings");
    expect_near(env.qti_white_jitter, 25.0f, 1e-6f,
        "QTI white response is randomized for training");
    expect_near(env.qti_black_jitter, 120.0f, 1e-6f,
        "QTI black response is randomized for training");
    expect_near(env.qti_threshold, 0.20f, 1e-6f,
        "weak real marker readings still count as line-seen in training");
    expect_near(env.line_width_jitter_m, 0.006f, 1e-6f,
        "episode line width randomization covers hand-drawn stripe thickness");
    expect_near(env.line_width_segment_jitter_m, 0.002f, 1e-6f,
        "per-track line width randomization covers uneven hand-drawn segments");
    expect_near(env.line_edge_softness_jitter_m, 0.004f, 1e-6f,
        "line edge softness is randomized for training");
    expect_near(env.line_reflectance_noise, 0.75f, 1e-6f,
        "line reflectance noise covers uneven hand-drawn marker darkness");
    expect_near(env.start_lateral_offset_m, 0.025f, 1e-6f,
        "start offset stays within the active sensor span");
    expect_near(env.start_heading_offset_rad, 0.261799f, 1e-6f,
        "start heading randomization covers about fifteen degrees");

    expect_near(env.sensor_forward[0], 0.0435f, 1e-6f, "left sensor forward default");
    expect_near(env.sensor_lateral[0], 0.018f, 1e-6f, "left sensor lateral default");
    expect_near(env.sensor_forward[1], 0.0435f, 1e-6f, "middle sensor forward default");
    expect_near(env.sensor_lateral[1], 0.0f, 1e-6f, "middle sensor lateral default");
    expect_near(env.sensor_forward[2], 0.0435f, 1e-6f, "right sensor forward default");
    expect_near(env.sensor_lateral[2], -0.018f, 1e-6f, "right sensor lateral default");

    env.sensor_lateral_center_m = -0.009f;
    env.sensor_lateral_jitter_m = 0.0f;
    sensor_layout(&env);
    expect_near(env.sensor_lateral[0], 0.009f, 1e-6f,
        "center offset can model the P6/P5/P4 active sensor trio");
    expect_near(env.sensor_lateral[1], -0.009f, 1e-6f,
        "center offset shifts the middle sensor consistently");
    expect_near(env.sensor_lateral[2], -0.027f, 1e-6f,
        "center offset shifts the right sensor consistently");
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
        expect_true(env.episode_dt >= 0.004f && env.episode_dt <= 0.009f,
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
    expect_near(env.dt, 0.0065f, 1e-6f, "H8 feed-forward timing matches measured fixed-point Prop C loop");
    expect_near(env.dt_min, 0.004f, 1e-5f, "H8 feed-forward timing preserves min ratio");
    expect_near(env.dt_max, 0.009f, 1e-5f, "H8 feed-forward timing preserves max ratio");
    expect_true(env.max_steps == 800, "H8 feed-forward keeps the configured wall-clock horizon");
    expect_true(env.lost_line_limit == 16, "H8 feed-forward keeps the configured lost-line grace");

    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.policy_hidden_size = 8;
    env.policy_num_layers = 1;
    apply_model_timing(&env);
    expect_near(env.dt, 0.192f, 1e-6f, "H8 recurrent timing uses measured slower loop");
    expect_near(env.dt_min, 0.118154f, 1e-5f, "H8 recurrent timing scales min dt");
    expect_near(env.dt_max, 0.265846f, 1e-5f, "H8 recurrent timing scales max dt");
    expect_true(env.max_steps == 28, "H8 recurrent reduces max steps to preserve wall-clock horizon");
    expect_true(env.lost_line_limit == 1, "H8 recurrent reduces lost-line grace to preserve wall-clock time");

    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.policy_hidden_size = 3;
    env.policy_num_layers = 1;
    apply_model_timing(&env);
    expect_near(env.dt, 0.0165f, 1e-6f, "small recurrent timing uses measured H4/L1 drive-loop budget");
    expect_near(env.dt_min, 0.010154f, 1e-5f, "small recurrent timing scales min dt");
    expect_near(env.dt_max, 0.022846f, 1e-5f, "small recurrent timing scales max dt");
    expect_true(env.max_steps == 316, "small recurrent keeps the configured wall-clock horizon");
    expect_true(env.lost_line_limit == 7, "small recurrent scales lost-line grace to preserve wall-clock time");

    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.policy_hidden_size = 4;
    env.policy_num_layers = 1;
    apply_model_timing(&env);
    expect_near(env.dt, 0.0165f, 1e-6f, "H4 recurrent timing shares the measured small-L1 budget");
    expect_true(env.max_steps == 316, "H4 recurrent keeps the configured wall-clock horizon");
    expect_true(env.lost_line_limit == 7, "H4 recurrent scales lost-line grace to preserve wall-clock time");

    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.policy_hidden_size = 4;
    env.policy_num_layers = 0;
    apply_model_timing(&env);
    expect_near(env.dt, 0.0065f, 1e-6f, "H4 feed-forward timing matches folded fixed-point inference cost");
    expect_true(env.max_steps == 800, "H4 feed-forward keeps the same folded-policy wall-clock horizon");
}

static void test_start_lateral_offset_samples_sensor_targets(void) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.rng = 123u;
    env.sensor_lateral_jitter_m = 0.0f;
    env.episode_line_width_m = 0.0f;
    sensor_layout(&env);

    bool seen[5] = {false};
    const float targets[5] = {
        -env.sensor_lateral[LINE_FOLLOW_LEFT],
        -0.5f * (env.sensor_lateral[LINE_FOLLOW_LEFT]
            + env.sensor_lateral[LINE_FOLLOW_MIDDLE]),
        -env.sensor_lateral[LINE_FOLLOW_MIDDLE],
        -0.5f * (env.sensor_lateral[LINE_FOLLOW_MIDDLE]
            + env.sensor_lateral[LINE_FOLLOW_RIGHT]),
        -env.sensor_lateral[LINE_FOLLOW_RIGHT],
    };

    for (int i = 0; i < 512; i++) {
        float offset = sample_start_lateral_offset(&env);
        expect_true(fabsf(offset) <= 0.025001f,
            "start sampler clamps lateral offset to twenty-five millimeters");
        for (int j = 0; j < 5; j++) {
            if (fabsf(offset - targets[j]) < 1e-6f) {
                seen[j] = true;
            }
        }
    }

    expect_true(seen[0], "start sampler can place left sensor on line");
    expect_true(seen[1], "start sampler can place line between left and middle sensors");
    expect_true(seen[2], "start sampler can place middle sensor on line");
    expect_true(seen[3], "start sampler can place line between middle and right sensors");
    expect_true(seen[4], "start sampler can place right sensor on line");
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

    expect_true(generate_arc(&track, 0.07f, 2.0f, 0.006f, 1.0f),
        "tight arc track generation succeeds");
    expect_true(track.length_m < 0.16f,
        "tight arc training case has a short-radius turn");
    expect_true(generate_oval(&track, 0.055f, 0.030f, 0.006f, 1.0f),
        "tight oval track generation succeeds");
    expect_true(track_in_bounds(&track, 1.0f),
        "tight oval track remains in bounds");
    expect_true(generate_corner(&track, 0.060f, 2.0f, 0.15f, 0.15f, 0.006f, 1.0f),
        "tight corner track generation succeeds");
    expect_true(track_in_bounds(&track, 1.0f),
        "tight corner track remains in bounds");
    expect_true(total_track_turn(&track) > 1.5f,
        "tight corner track contains a sharp heading transition");

    unsigned int rng = 123u;
    bool seen_positive_start_y = false;
    bool seen_negative_start_y = false;
    bool seen_clockwise = false;
    bool seen_counter_clockwise = false;
    bool seen_family[5] = {false};
    for (unsigned int seed = 1; seed < 512; seed++) {
        rng = seed;
        expect_true(generate_track(&track, &rng, LINE_FOLLOW_TRACK_RANDOM, 0.006f, 1.0f),
            "randomized training track generation succeeds");
        expect_true(track.sample_count <= LINE_FOLLOW_MAX_TRACK_SAMPLES,
            "randomized generation respects sample cap");
        if (track.family >= 0 && track.family < 5) {
            seen_family[track.family] = true;
        }
        float start_dy = track.samples[1].y - track.samples[0].y;
        if (start_dy > 1e-5f) {
            seen_positive_start_y = true;
        }
        if (start_dy < -1e-5f) {
            seen_negative_start_y = true;
        }
        float turn = total_track_turn(&track);
        if (turn > 0.2f) {
            seen_counter_clockwise = true;
        }
        if (turn < -0.2f) {
            seen_clockwise = true;
        }
    }
    expect_true(seen_positive_start_y && seen_negative_start_y,
        "randomized training tracks include both left and right handed starts");
    expect_true(seen_clockwise && seen_counter_clockwise,
        "randomized training tracks include both clockwise and counter-clockwise turns");
    expect_true(seen_family[LINE_FOLLOW_TRACK_STRAIGHT]
            && seen_family[LINE_FOLLOW_TRACK_ARC]
            && seen_family[LINE_FOLLOW_TRACK_S_CURVE]
            && seen_family[LINE_FOLLOW_TRACK_OVAL]
            && seen_family[LINE_FOLLOW_TRACK_CORNER],
        "randomized training distribution covers straight, curved, and corner families");
}

static void test_episode_progress_starts_at_zero(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    int families[] = {
        LINE_FOLLOW_TRACK_STRAIGHT,
        LINE_FOLLOW_TRACK_ARC,
        LINE_FOLLOW_TRACK_S_CURVE,
        LINE_FOLLOW_TRACK_OVAL,
        LINE_FOLLOW_TRACK_CORNER,
        LINE_FOLLOW_TRACK_RANDOM,
    };

    for (int family_idx = 0; family_idx < 6; family_idx++) {
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
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    expect_true(generate_straight(&env.track, 1.0f, env.line_width_m, env.track_bounds_m),
        "perf test track generation succeeds");
    env.episode_line_width_m = env.line_width_m;

    env.tick = 10;
    env.episode_progress = perf_target_progress_m(&env);
    env.effective_progress_m = env.episode_progress;
    env.center_path_m = env.episode_progress;
    int target_checkpoints = perf_target_checkpoints(&env);
    env.perf_quality_sum = (float)target_checkpoints;
    env.perf_checkpoint_count = target_checkpoints;
    env.centerline_error = 0.0f;
    env.centerline_error_sum = 0.0f;
    env.forward_speed_sum = env.avg_speed_perf_target_mps * (float)env.tick;
    add_log(&env);
    expect_near(env.log.base_perf, 1.0f, 1e-6f,
        "full accuracy-weighted progress gives full base perf");
    expect_near(env.log.min_drive_speed_score, 0.3f, 1e-6f,
        "default six tick anti-stall floor is logged as a hardware-floor diagnostic");
    expect_near(env.log.avg_speed_score, 1.0f, 1e-6f,
        "target average speed gives full average-speed score");
    expect_near(env.log.perf, 1.0f, 1e-6f,
        "default perf is base perf scaled by actual average speed");
    expect_near(env.log.score, 0.0f, 1e-6f,
        "score logs raw episode return separately from perf");
    expect_near(env.log.progress_frac, 1.0f, 1e-6f,
        "full target progress logs full progress fraction");
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
    expect_near(env.log.effective_progress_m, env.effective_progress_m, 1e-6f,
        "accuracy-weighted progress meters are logged");
    expect_near(env.log.center_path_m, env.center_path_m, 1e-6f,
        "axle-center path meters are logged");
    expect_near(env.log.path_efficiency, 1.0f, 1e-6f,
        "centered straight progress has full path efficiency");

    memset(&env.log, 0, sizeof(env.log));
    env.path_efficiency_perf_weight = 0.4f;
    env.center_path_m = 2.0f * env.episode_progress;
    add_log(&env);
    expect_near(env.log.path_efficiency, 0.5f, 1e-6f,
        "path efficiency logs raw progress divided by axle-center travel");
    expect_near(env.log.perf, 0.8f, 1e-6f,
        "path-efficiency perf weight discounts inefficient paths without hard-coding sensors");
    env.path_efficiency_perf_weight = 0.0f;
    env.center_path_m = env.episode_progress;

    memset(&env.log, 0, sizeof(env.log));
    env.min_drive_ticks_per_sec = 20.0f;
    add_log(&env);
    expect_near(env.log.perf, 1.0f, 1e-6f,
        "minimum drive floor no longer changes perf when average speed is good");
    expect_near(env.log.min_drive_speed_score, 1.0f, 1e-6f,
        "twenty tick minimum drive floor is still logged for diagnostics");

    memset(&env.log, 0, sizeof(env.log));
    env.min_drive_ticks_per_sec = 6.0f;
    env.forward_speed_sum = 0.5f * env.avg_speed_perf_target_mps * (float)env.tick;
    add_log(&env);
    expect_near(env.log.avg_speed_score, 0.5f, 1e-6f,
        "half target average speed gives half speed score");
    expect_near(env.log.perf, 0.5f, 1e-6f,
        "average-speed score, not hardware floor, scales perf");

    memset(&env.log, 0, sizeof(env.log));
    env.min_drive_ticks_per_sec = 6.0f;
    env.tick = 10;
    env.episode_progress = perf_target_progress_m(&env);
    env.effective_progress_m = 0.0f;
    env.center_path_m = env.episode_progress;
    env.perf_quality_sum = (float)target_checkpoints;
    env.perf_checkpoint_count = target_checkpoints;
    env.centerline_error = 2.0f * env.sensor_side_lateral_m;
    env.centerline_error_sum = 2.0f * env.sensor_side_lateral_m * 10.0f;
    env.forward_speed_sum = env.avg_speed_perf_target_mps * (float)env.tick;
    add_log(&env);
    expect_near(env.log.perf, 0.0f, 1e-6f,
        "zero effective progress gives zero perf even with full raw progress");

    memset(&env.log, 0, sizeof(env.log));
    env.tick = 10;
    env.episode_progress = perf_target_progress_m(&env);
    env.effective_progress_m = env.episode_progress;
    env.center_path_m = env.episode_progress;
    env.perf_quality_sum = (float)target_checkpoints;
    env.perf_checkpoint_count = target_checkpoints;
    env.centerline_error = 0.0f;
    env.centerline_error_sum = 0.0f;
    env.min_drive_ticks_per_sec = 20.0f;
    env.terminal_lost_line = 1.0f;
    env.episode_return = 42.0f;
    add_log(&env);
    expect_near(env.log.perf, 0.0f, 1e-6f,
        "lost-line terminal zeroes perf despite prior good progress");
    expect_near(env.log.score, -1.0f, 1e-6f,
        "lost-line terminal logs the run score as the off-track penalty");
    expect_near(env.log.episode_return, -1.0f, 1e-6f,
        "lost-line terminal logs the run return as the off-track penalty");
    expect_near(env.log.base_perf, 0.0f, 1e-6f,
        "lost-line terminal zeroes base perf despite prior good progress");
    expect_near(env.log.progress_frac, 0.0f, 1e-6f,
        "lost-line terminal zeroes progress score despite prior good progress");
    expect_near(env.log.checkpoint_accuracy, 0.0f, 1e-6f,
        "lost-line terminal zeroes checkpoint accuracy for sweep selection");
    expect_near(env.log.effective_progress_m, 0.0f, 1e-6f,
        "lost-line terminal zeroes scored effective progress meters");
    expect_near(env.log.progress_m, env.episode_progress, 1e-6f,
        "lost-line terminal still logs raw progress meters for debugging");
    env.terminal_lost_line = 0.0f;
}

static void test_continuous_progress_quality(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    env.episode_line_width_m = env.line_width_m;

    expect_near(accuracy_score(&env, 0.000f), 1.0f, 1e-6f,
        "zero millimeter centerline error has full accuracy");
    expect_near(accuracy_score(&env, 0.005f), 5.0f / 6.0f, 1e-6f,
        "five millimeter centerline error is linearly scored");
    expect_near(accuracy_score(&env, 0.010f), 2.0f / 3.0f, 1e-6f,
        "ten millimeter centerline error is linearly scored");
    expect_near(accuracy_score(&env, 0.015f), 0.5f, 1e-6f,
        "fifteen millimeter centerline error has half accuracy");
    expect_near(accuracy_score(&env, 0.020f), 1.0f / 3.0f, 1e-6f,
        "twenty millimeter centerline error is linearly scored");
    expect_near(accuracy_score(&env, 0.030f), 0.0f, 1e-6f,
        "thirty millimeter centerline error has zero accuracy");
    expect_near(reward_centerline_score(&env, 0.000f), 1.0f, 1e-6f,
        "reward centerline score is full at zero error");
    expect_near(reward_centerline_score(&env, 0.015f), 0.0f, 1e-6f,
        "reward centerline score is zero before the line sits under an inner sensor");
    expect_near(reward_centerline_score(&env, 0.030f), -1.0f, 1e-6f,
        "reward centerline score is minus one at thirty millimeters");

    float first = record_progress_metrics(&env, 0.010f, 0.0f);
    expect_near(first, 0.0f, 1e-6f,
        "progress metric recording no longer emits checkpoint reward");
    expect_near(env.effective_progress_m, 0.010f, 1e-6f,
        "centered progress adds full effective progress");
    expect_near(env.perf_quality_sum, 1.0f, 1e-6f,
        "centered checkpoint debug adds full perf quality");
    expect_true(env.perf_checkpoint_count == 1,
        "centered checkpoint debug increments perf checkpoint count");

    record_progress_metrics(&env, 0.010f, 0.020f);
    expect_near(env.effective_progress_m, 0.010f + 0.010f / 3.0f, 1e-6f,
        "off-center progress adds linearly reduced effective progress");
    expect_near(env.perf_quality_sum, 4.0f / 3.0f, 1e-6f,
        "twenty millimeter checkpoint debug adds its linear perf quality");
    expect_true(env.perf_checkpoint_count == 2,
        "second checkpoint debug increments perf checkpoint count");

    float far_training_reward = record_progress_metrics(&env, 0.010f, 0.060f);
    expect_near(far_training_reward, 0.0f, 1e-6f,
        "far off-center progress metric does not emit training reward");
    expect_near(env.perf_quality_sum, 4.0f / 3.0f, 1e-6f,
        "far checkpoints do not reduce bounded debug quality below zero");
    expect_near(env.effective_progress_m, 0.010f + 0.010f / 3.0f, 1e-6f,
        "far progress adds no effective progress");
}

static void test_progress_rejects_discontinuous_nearest_jump(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
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

static void test_continuous_progress_and_motion_rewards(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);

    float first = record_progress_metrics(&env, 0.009f, 0.0f);
    expect_near(first, 0.0f, 1e-6f,
        "progress metric recording does not emit milestone reward");
    float second = record_progress_metrics(&env, 0.001f, 0.0f);
    expect_near(second, 0.0f, 1e-6f,
        "progress metric recording stays reward-free at ten millimeters");
    expect_near(env.centerline_reward_progress_m, 0.0f, 1e-6f,
        "checkpoint debug accumulator subtracts counted interval");
    expect_near(env.effective_progress_m, 0.010f, 1e-6f,
        "continuous effective progress tracks every millimeter");

    float before_far = env.effective_progress_m;
    record_progress_metrics(&env, 0.010f, 2.0f * env.sensor_side_lateral_m);
    expect_near(env.effective_progress_m, before_far, 1e-6f,
        "far-off progress adds no effective progress");

    float max_tick_motion = env.max_wheel_speed_mps * env.dt;
    expect_near(
        normalized_wasted_motion(&env, max_tick_motion, max_tick_motion),
        0.0f, 1e-6f,
        "motion that becomes track progress is not wasted");
    expect_near(
        normalized_wasted_motion(&env, max_tick_motion, 0.25f * max_tick_motion),
        0.75f, 1e-6f,
        "path motion without matching progress is normalized as wasted motion");
    expect_near(
        lost_line_recovery_score(&env, 0.030f, 0.012f),
        1.0f, 1e-6f,
        "lost-line recovery score rewards shrinking centerline error");
    expect_near(
        lost_line_recovery_score(&env, 0.012f, 0.030f),
        -1.0f, 1e-6f,
        "lost-line recovery score penalizes moving farther from the centerline");

    float stopped = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, true, 0.0f, 0.0f);
    expect_near(stopped, -env.time_penalty - env.idle_penalty, 1e-6f,
        "centered stopped robot gets time and idle penalties");

    float straight = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f);
    float turning = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, false, 0.0f, 0.0f);
    expect_near(straight, turning, 1e-6f,
        "turning is not directly penalized");
    env.action_smoothness_penalty = 0.05f;
    float action_jump = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.0f,
        2.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f);
    expect_near(action_jump, straight - 0.10f, 1e-6f,
        "action smoothness penalty subtracts reward for command jumps");
    env.action_smoothness_penalty = 0.0f;
    float saved_steering_correction_scale = env.steering_correction_scale;
    env.steering_correction_scale = 0.0f;
    env.turn_penalty_scale = 0.05f;
    float centered_turn_penalty = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, false, 0.0f, 0.0f);
    float edge_turn_penalty = compute_reward(&env, 0.0f, env.sensor_side_lateral_m,
        0.0f, true, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, false, 0.0f, 0.0f);
    expect_near(centered_turn_penalty, straight - env.turn_penalty_scale, 1e-6f,
        "centered turn penalty discourages spinning on the middle sensor");
    expect_near(edge_turn_penalty, straight, 1e-6f,
        "centered turn penalty fades out at side-sensor error");
    env.turn_penalty_scale = 0.0f;
    env.ambiguous_turn_penalty_scale = 0.10f;
    env.sensor_obs[LINE_FOLLOW_LEFT] = 0.0f;
    env.sensor_obs[LINE_FOLLOW_MIDDLE] = 0.20f;
    env.sensor_obs[LINE_FOLLOW_RIGHT] = 0.0f;
    float weak_center_weight = ambiguous_observation_turn_weight(&env);
    float weak_center_turn_penalty = compute_reward(&env, 0.0f, 0.0f, 0.0f,
        true, 0.0f, 0.0f, 0.0f, weak_center_weight, 1.0f, false, 0.0f, 0.0f);
    env.sensor_obs[LINE_FOLLOW_LEFT] = 0.80f;
    env.sensor_obs[LINE_FOLLOW_MIDDLE] = 0.20f;
    env.sensor_obs[LINE_FOLLOW_RIGHT] = 0.0f;
    float lateral_evidence_weight = ambiguous_observation_turn_weight(&env);
    float lateral_evidence_turn = compute_reward(&env, 0.0f, 0.0f, 0.0f,
        true, 0.0f, 0.0f, 0.0f, lateral_evidence_weight, 1.0f, false, 0.0f, 0.0f);
    env.sensor_obs[LINE_FOLLOW_LEFT] = 0.0f;
    env.sensor_obs[LINE_FOLLOW_MIDDLE] = 0.0f;
    env.sensor_obs[LINE_FOLLOW_RIGHT] = 0.0f;
    float all_white_weight = ambiguous_observation_turn_weight(&env);
    float all_white_turn = compute_reward(&env, 0.0f, 0.0f, 0.0f,
        true, 0.0f, 0.0f, 0.0f, all_white_weight, 1.0f, false, 0.0f, 0.0f);
    expect_near(weak_center_turn_penalty,
        straight - env.ambiguous_turn_penalty_scale, 1e-6f,
        "ambiguous visible line observations discourage invented turns");
    expect_near(lateral_evidence_turn, straight, 1e-6f,
        "ambiguous turn penalty does not suppress asymmetric side evidence");
    expect_near(all_white_turn, straight, 1e-6f,
        "ambiguous turn penalty does not hard-code all-white recovery");
    env.ambiguous_turn_penalty_scale = 0.0f;
    env.steering_correction_scale = saved_steering_correction_scale;
    float correcting_right = compute_reward(&env, 0.0f, 0.020f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 0.0f, -1.0f, false, 0.0f, 0.0f);
    float turning_left_when_right_needed = compute_reward(&env, 0.0f, 0.020f, 0.0f, true, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, false, 0.0f, 0.0f);
    expect_true(correcting_right > turning_left_when_right_needed,
        "privileged centerline error rewards turning toward the line");

    float full_tick_progress = env.max_wheel_speed_mps * env.dt;
    float half_tick_progress = 0.5f * full_tick_progress;

    float pre_checkpoint = compute_reward(&env, half_tick_progress, 0.0f, 0.0f, true, 0.1f,
        0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f);
    expect_near(pre_checkpoint, 0.5f * env.progress_reward_scale - env.time_penalty, 1e-6f,
        "forward progress receives normalized dense reward before a checkpoint");

    float full_progress_centered = compute_reward(&env, full_tick_progress, 0.0f, 0.0f, true, 0.1f,
        0.0f, reward_centerline_score(&env, 0.0f), 0.0f, 0.0f, false, 0.0f, 0.0f);
    float full_progress_off_center = compute_reward(&env, full_tick_progress, 0.020f, 0.0f, true, 0.1f,
        0.0f, reward_centerline_score(&env, 0.020f), 0.0f, 0.0f, false, 0.0f, 0.0f);
    expect_true(full_progress_centered > full_progress_off_center,
        "continuous progress reward is scaled by centerline quality");
    expect_near(full_progress_centered,
        env.progress_reward_scale - env.time_penalty,
        1e-6f,
        "full-tick centered reward is normalized progress minus time penalty");

    float bounded = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.1f,
        0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f);
    float out_of_bounds = compute_reward(&env, 0.0f, 0.0f, 0.0f, true, 0.1f,
        0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 2.0f);
    expect_true(out_of_bounds < bounded,
        "raw actions outside the policy contract receive a soft penalty");
}

static void test_sensor_and_reward_behavior(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    expect_true(generate_straight(&env.track, 1.0f, env.line_width_m, env.track_bounds_m),
        "test straight track generation succeeds");
    reset_runtime_pose(&env, 0.0f, 0.0f, 0.0f);
    compute_observations(&env);
    env.centerline_error = 0.0f;

    expect_near(obs[0], obs[2], 1e-5f,
        "centered side QTI readings are symmetric");
    expect_true(obs[0] <= 0.01f && obs[1] >= 0.99f && obs[2] <= 0.01f,
        "centered measured sensor layout puts the middle sensor on the line");
    expect_true(line_visible_to_sensor_array(&env),
        "centered line under the middle sensor is visible to the array");
    expect_true(line_seen_by_qti(&env),
        "centered line under the middle sensor is visible in QTI observations");

    reset_runtime_pose(&env, 0.0f, 0.02f, 0.0f);
    compute_observations(&env);
    env.centerline_error = 0.02f;
    expect_true(obs[2] > obs[0] && obs[2] > obs[1],
        "left-shifted robot makes right QTI reading darker");

    reset_runtime_pose(&env, 0.0f, -0.02f, 0.0f);
    compute_observations(&env);
    expect_true(obs[0] > obs[1] && obs[0] > obs[2],
        "right-shifted robot makes left QTI reading darker");

    reset_runtime_pose(&env, 0.0f, 0.09f, 0.0f);
    compute_observations(&env);
    expect_true(!line_visible_to_sensor_array(&env),
        "line fully right of the right sensor is treated as lost");

    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        env.sensor_obs[i] = 0.0f;
        env.observations[i] = 0.0f;
    }
    expect_true(!line_seen_by_qti(&env),
        "all-white QTI observations are treated as not seeing the line");

    reset_runtime_pose(&env, 0.0f, -0.09f, 0.0f);
    compute_observations(&env);
    expect_true(!line_visible_to_sensor_array(&env),
        "line fully left of the left sensor is treated as lost");

    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        obs[i] = 0.5f;
    }
    float near_bonus = accuracy_score(&env, 0.0f);
    float far_bonus = accuracy_score(&env, 0.04f);
    float near_reward = compute_reward(&env, 0.025f, 0.0f, 0.0f, true, 0.1f,
        0.0f, near_bonus, 0.0f, 0.0f, false, 0.0f, 0.0f);
    float far_reward = compute_reward(&env, 0.025f, 0.04f, 0.0f, true, 0.1f,
        0.0f, far_bonus, 0.0f, 0.0f, false, 0.0f, 0.0f);
    expect_true(near_reward > far_reward,
        "reward changes with privileged centerline distance even when observations are ambiguous");
}

static void test_negative_wheel_command_is_penalized_without_terminal(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
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

    expect_near(env.v_left_cmd,
        wheel_ticks_per_sec_to_mps(env.min_drive_ticks_per_sec, env.tire_diameter_m),
        1e-6f,
        "negative normalized left action maps to the anti-stall wheel floor in sim");
    expect_true(env.v_right_cmd > env.v_left_cmd,
        "positive normalized right action maps faster than the left wheel");
    expect_near(terminals[0], 0.0f, 1e-6f,
        "negative wheel command is no longer an immediate terminal");
    expect_near(env.log.n, 0.0f, 1e-6f,
        "negative wheel command does not log a terminal episode by itself");
    expect_true(env.negative_action_steps == 1,
        "negative normalized action is logged and penalized without forcing a terminal");
    expect_true(rewards[0] >= -1.0f && rewards[0] <= 1.0f,
        "slow wheel command receives a finite clamped dense reward");
}

static void reset_centered_straight_episode(LineFollow* env);

static void test_raw_action_out_of_bounds_is_soft_penalized(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {1.01f, 1.01f};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    reset_centered_straight_episode(&env);

    c_step(&env);

    expect_near(actions[0], 1.0f, 1e-6f,
        "raw action above upper bound is clamped for downstream env state");
    expect_near(actions[1], 1.0f, 1e-6f,
        "second raw action above upper bound is clamped for downstream env state");
    expect_near(terminals[0], 0.0f, 1e-6f,
        "raw action above upper bound is clipped and does not terminal");
    expect_true(rewards[0] >= -1.0f && rewards[0] <= 1.0f,
        "raw action above upper bound receives a finite clipped reward");
    expect_near(env.log.n, 0.0f, 1e-6f,
        "raw action upper bound violation does not log a terminal episode");
    expect_true(env.action_bound_violation_sum > 0.0f,
        "raw action upper bound violation is accumulated for logging");
    expect_near(env.log.terminal_negative, 0.0f, 1e-6f,
        "raw action upper bound violation is not logged as a negative-action terminal");

    actions[0] = 0.0f;
    actions[1] = -1.01f;
    rewards[0] = 0.0f;
    terminals[0] = 0.0f;
    memset(&env.log, 0, sizeof(env.log));
    reset_centered_straight_episode(&env);

    c_step(&env);

    expect_near(actions[1], -1.0f, 1e-6f,
        "raw action below lower bound is clamped for downstream env state");
    expect_near(terminals[0], 0.0f, 1e-6f,
        "raw action below lower bound is clipped and does not terminal");
    expect_true(rewards[0] >= -1.0f && rewards[0] <= 1.0f,
        "raw action below lower bound receives a finite clipped reward");
    expect_near(env.log.n, 0.0f, 1e-6f,
        "raw action lower bound violation does not log a terminal episode");
    expect_true(env.action_bound_violation_sum > 0.0f,
        "raw action lower bound violation is accumulated for logging");
    expect_near(env.log.terminal_negative, 0.0f, 1e-6f,
        "raw action lower bound violation is not logged as a negative-action terminal");
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
    env->effective_progress_m = 0.0f;
    env->center_path_m = 0.0f;
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
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {-1.0f, -1.0f};
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
    expect_true(rewards[0] <= 1.0f && rewards[0] >= -1.0f,
        "moving timeout reward is clipped to the policy reward range");
    expect_true(moving_return > 0.0f,
        "moving timeout can earn positive progress-based return");

    actions[0] = -1.0f;
    actions[1] = -1.0f;
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

static void test_off_track_terminal_penalty(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    env.lost_line_limit = 1;
    env.too_far_m = 10.0f;
    env.off_track_terminal_penalty = 0.75f;
    reset_centered_straight_episode(&env);
    env.episode_return = 12.0f;
    reset_runtime_pose(&env, env.track.samples[0].x,
        env.track.samples[0].y + 0.08f, 0.0f);
    compute_observations(&env);

    c_step(&env);

    expect_near(terminals[0], 1.0f, 1e-6f,
        "lost line emits a terminal flag");
    expect_near(rewards[0], -0.75f, 1e-6f,
        "lost line terminal reward is the configured negative penalty");
    expect_near(env.log.terminal_lost_line, 1.0f, 1e-6f,
        "lost line terminal is logged");
    expect_near(env.log.episode_return, -0.75f, 1e-6f,
        "logged return is exactly the terminal off-track penalty");
    expect_near(env.log.score, -0.75f, 1e-6f,
        "logged score is exactly the terminal off-track penalty");
    expect_near(env.log.perf, 0.0f, 1e-6f,
        "lost line terminal logs zero perf");
}

static void test_reward_clamp_and_reset_start_visibility(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);
    env.rng = 456u;
    env.track_family = LINE_FOLLOW_TRACK_RANDOM;

    for (int episode = 0; episode < 256; episode++) {
        c_reset(&env);
        expect_true(line_visible_to_sensor_array(&env),
            "random reset starts with the line inside the active sensor span");
        expect_true(env.track.family >= LINE_FOLLOW_TRACK_STRAIGHT
                && env.track.family <= LINE_FOLLOW_TRACK_CORNER,
            "random reset chooses a supported training track family");
    }

    expect_true(generate_straight(&env.track, 1.0f, env.line_width_m, env.track_bounds_m),
        "reward clamp test straight track generation succeeds");
    env.episode_line_width_m = env.line_width_m;
    env.episode_line_edge_softness_m = env.line_edge_softness_m;
    env.time_penalty = 100.0f;
    env.lost_line_limit = 100;
    reset_runtime_pose(&env, env.track.samples[0].x, env.track.samples[0].y, 0.0f);
    compute_observations(&env);

    c_step(&env);

    expect_true(rewards[0] >= -1.0f && rewards[0] <= 1.0f,
        "emitted step reward is clamped to [-1, 1]");
}

static void test_randomized_sensor_response_clamps_observations(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);

    env.qti_white_jitter = 200.0f;
    env.qti_black_jitter = 3000.0f;
    env.sensor_noise_std = 2000.0f;
    env.line_width_jitter_m = 0.010f;
    env.line_width_segment_jitter_m = 0.010f;
    env.line_edge_softness_jitter_m = 0.010f;
    env.line_reflectance_noise = 0.50f;

    for (int episode = 0; episode < 32; episode++) {
        c_reset(&env);
        for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
            expect_true(obs[i] >= 0.0f && obs[i] <= 1.0f,
                "randomized reset observations stay clamped to [0, 1]");
        }
        for (int i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++) {
            expect_true(env.sensor_obs[i] >= 0.0f && env.sensor_obs[i] <= 1.0f,
                "randomized reset sensor telemetry stays clamped to [0, 1]");
            expect_true(env.sensor_black_time[i] > env.sensor_white_time[i],
                "randomized QTI black calibration stays above white calibration");
        }
        expect_true(env.episode_line_width_m >= 0.004f,
            "randomized line width stays positive");
        for (int i = 0; i < env.track.sample_count; i++) {
            expect_true(env.track.samples[i].line_width_m >= 0.004f,
                "randomized per-segment line width stays positive");
            expect_true(env.track.samples[i].black_reflectance >= 0.50f,
                "randomized per-segment line reflectance stays dark enough");
            expect_true(env.track.samples[i].black_reflectance <= 1.0f,
                "randomized per-segment line reflectance does not exceed black calibration");
        }
        expect_true(env.episode_line_edge_softness_m >= 0.001f,
            "randomized line edge softness stays positive");
        actions[0] = rand_signed(&env.rng);
        actions[1] = rand_signed(&env.rng);
        c_step(&env);
        for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
            expect_true(obs[i] >= 0.0f && obs[i] <= 1.0f,
                "randomized step observations stay clamped to [0, 1]");
        }
        for (int i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++) {
            expect_true(env.sensor_obs[i] >= 0.0f && env.sensor_obs[i] <= 1.0f,
                "randomized step sensor telemetry stays clamped to [0, 1]");
        }
    }
}

static void test_segment_line_width_jitter_varies_track_samples(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);

    env.rng = 123u;
    env.track_family = LINE_FOLLOW_TRACK_S_CURVE;
    env.line_width_jitter_m = 0.0f;
    env.line_width_segment_jitter_m = 0.002f;
    c_reset(&env);

    float min_width = 1e9f;
    float max_width = -1e9f;
    for (int i = 0; i < env.track.sample_count; i++) {
        float width = env.track.samples[i].line_width_m;
        if (width < min_width) min_width = width;
        if (width > max_width) max_width = width;
    }

    expect_true(max_width - min_width > 0.0001f,
        "per-segment width jitter changes line thickness inside an episode");
    expect_true(min_width >= env.line_width_m - env.line_width_segment_jitter_m - 1e-6f,
        "per-segment width jitter respects the lower configured range");
    expect_true(max_width <= env.line_width_m + env.line_width_segment_jitter_m + 1e-6f,
        "per-segment width jitter respects the upper configured range");
}

static void test_line_reflectance_noise_varies_track_samples(void) {
    float obs[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[2] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    LineFollow env = make_test_env(obs, actions, rewards, terminals);

    env.rng = 456u;
    env.track_family = LINE_FOLLOW_TRACK_S_CURVE;
    env.line_reflectance_noise = 0.75f;
    c_reset(&env);

    float min_reflectance = 1e9f;
    float max_reflectance = -1e9f;
    for (int i = 0; i < env.track.sample_count; i++) {
        float reflectance = env.track.samples[i].black_reflectance;
        if (reflectance < min_reflectance) min_reflectance = reflectance;
        if (reflectance > max_reflectance) max_reflectance = reflectance;
    }

    expect_true(max_reflectance - min_reflectance > 0.003f,
        "line reflectance noise changes marker darkness inside an episode");
    expect_true(min_reflectance >= 0.25f - 1e-6f,
        "line reflectance noise respects the lower configured range");
    expect_true(max_reflectance <= 1.0f + 1e-6f,
        "line reflectance noise respects the upper configured range");

    env = make_test_env(obs, actions, rewards, terminals);
    env.sensor_forward_jitter_m = 0.0f;
    env.sensor_lateral_jitter_m = 0.0f;
    env.qti_white_jitter = 0.0f;
    env.qti_black_jitter = 0.0f;
    env.sensor_noise_std = 0.0f;
    env.episode_line_width_m = env.line_width_m;
    env.episode_line_edge_softness_m = env.line_edge_softness_m;
    sensor_layout(&env);
    expect_true(generate_straight(&env.track, 1.0f, env.line_width_m, env.track_bounds_m),
        "straight reflectance test track generation succeeds");
    reset_runtime_pose(&env, 0.0f, 0.0f, 0.0f);
    compute_observations(&env);
    float full_black_middle = env.sensor_obs[LINE_FOLLOW_MIDDLE];

    for (int i = 0; i < env.track.sample_count; i++) {
        env.track.samples[i].black_reflectance = 0.50f;
    }
    compute_observations(&env);
    float weak_black_middle = env.sensor_obs[LINE_FOLLOW_MIDDLE];

    expect_true(full_black_middle > 0.95f,
        "full black reflectance produces a black middle-sensor observation");
    expect_true(weak_black_middle > 0.45f && weak_black_middle < 0.55f,
        "weaker black reflectance scales the middle-sensor observation");
    expect_true(weak_black_middle < full_black_middle - 0.40f,
        "weaker black reflectance is materially different from full black");
}

int main(void) {
    test_qti_normalization();
    test_propeller_qti_normalization_parity();
    test_rawdiff_observation_mapping();
    test_continuous_actions();
    test_measured_geometry_defaults();
    test_episode_dt_randomization();
    test_model_timing_penalizes_larger_policy();
    test_start_lateral_offset_samples_sensor_targets();
    test_track_generation();
    test_episode_progress_starts_at_zero();
    test_perf_penalizes_centerline_error();
    test_continuous_progress_quality();
    test_progress_rejects_discontinuous_nearest_jump();
    test_continuous_progress_and_motion_rewards();
    test_sensor_and_reward_behavior();
    test_negative_wheel_command_is_penalized_without_terminal();
    test_raw_action_out_of_bounds_is_soft_penalized();
    test_timeout_success_reward_requires_progress();
    test_off_track_terminal_penalty();
    test_reward_clamp_and_reset_start_visibility();
    test_randomized_sensor_response_clamps_observations();
    test_segment_line_width_jitter_varies_track_samples();
    test_line_reflectance_noise_varies_track_samples();

    if (failures > 0) {
        fprintf(stderr, "%d line_follow test failures\n", failures);
        return 1;
    }

    printf("line_follow tests passed\n");
    return 0;
}
