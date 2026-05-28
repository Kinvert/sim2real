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

static void test_continuous_actions(void) {
    float actions[2] = {-2.0f, 2.0f};
    LineFollowWheelCommand cmd = map_actions(actions, 0.5f, 0.05f, 1.0f, 1.0f);
    expect_near(actions[0], -1.0f, 1e-6f, "left action clamps low");
    expect_near(actions[1], 1.0f, 1e-6f, "right action clamps high");
    expect_near(cmd.left_mps, -0.5f, 1e-6f, "clamped left action maps to negative wheel speed");
    expect_near(cmd.right_mps, 0.5f, 1e-6f, "clamped right action maps to positive wheel speed");

    float tiny[2] = {0.02f, -0.04f};
    cmd = map_actions(tiny, 0.5f, 0.05f, 1.0f, 1.0f);
    expect_near(cmd.left_mps, 0.0f, 1e-6f, "left deadband maps tiny command to zero");
    expect_near(cmd.right_mps, 0.0f, 1e-6f, "right deadband maps tiny command to zero");

    float polarity[2] = {1.0f, -1.0f};
    cmd = map_actions(polarity, 0.35f, 0.0f, 1.0f, 1.0f);
    expect_near(cmd.left_mps, 0.35f, 1e-6f, "positive left command drives left wheel forward");
    expect_near(cmd.right_mps, -0.35f, 1e-6f, "negative right command drives right wheel backward");
}

static void test_measured_geometry_defaults(void) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);

    expect_near(env.wheel_base_m, 0.125f, 1e-6f, "measured tire-to-tire width is default wheel base");
    expect_near(env.body_ahead_m, 0.040f, 1e-6f, "measured body ahead of axle is default");
    expect_near(env.body_behind_m, 0.090f, 1e-6f, "measured body behind axle is default");
    expect_near(env.tire_diameter_m, 0.065f, 1e-6f, "measured tire diameter is default");
    expect_near(env.sensor_forward_m, 0.035f, 1e-6f, "measured sensor forward offset is default");
    expect_near(env.sensor_inner_lateral_m, 0.020f, 1e-6f,
        "measured inner sensor lateral offset is default");
    expect_near(env.sensor_outer_lateral_m, 0.040f, 1e-6f,
        "measured outer sensor lateral offset is default");

    expect_near(env.sensor_forward[0], 0.035f, 1e-6f, "outer-left sensor forward default");
    expect_near(env.sensor_lateral[0], 0.040f, 1e-6f, "outer-left sensor lateral default");
    expect_near(env.sensor_forward[1], 0.035f, 1e-6f, "inner-left sensor forward default");
    expect_near(env.sensor_lateral[1], 0.020f, 1e-6f, "inner-left sensor lateral default");
    expect_near(env.sensor_forward[2], 0.035f, 1e-6f, "inner-right sensor forward default");
    expect_near(env.sensor_lateral[2], -0.020f, 1e-6f, "inner-right sensor lateral default");
    expect_near(env.sensor_forward[3], 0.035f, 1e-6f, "outer-right sensor forward default");
    expect_near(env.sensor_lateral[3], -0.040f, 1e-6f, "outer-right sensor lateral default");
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

    expect_near(obs[0], obs[3], 1e-5f, "centered outer QTI readings are symmetric");
    expect_near(obs[1], obs[2], 1e-5f, "centered inner QTI readings are symmetric");
    expect_true(obs[0] <= 0.01f && obs[1] <= 0.01f && obs[2] <= 0.01f && obs[3] <= 0.01f,
        "centered measured sensors straddle the narrow default line");

    reset_runtime_pose(&env, 0.0f, 0.02f, 0.0f);
    compute_observations(&env);
    float left_darkness = obs[0] + obs[1];
    float right_darkness = obs[2] + obs[3];
    expect_true(right_darkness > left_darkness, "left-shifted robot makes right QTI readings darker");

    for (int i = 0; i < 4; i++) {
        obs[i] = 0.5f;
    }
    float near_reward = compute_reward(&env, 0.02f, 0.0f, 0.0f, true, 0.1f, 0.0f);
    float far_reward = compute_reward(&env, 0.02f, 0.04f, 0.0f, true, 0.1f, 0.0f);
    expect_true(near_reward > far_reward,
        "reward changes with privileged centerline distance even when observations are ambiguous");
}

int main(void) {
    test_qti_normalization();
    test_continuous_actions();
    test_measured_geometry_defaults();
    test_track_generation();
    test_sensor_and_reward_behavior();

    if (failures > 0) {
        fprintf(stderr, "%d line_follow test failures\n", failures);
        return 1;
    }

    printf("line_follow tests passed\n");
    return 0;
}
