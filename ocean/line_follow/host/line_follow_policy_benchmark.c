#define LINE_FOLLOW_NO_RENDER

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ocean/line_follow/line_follow.h"
#include "src/puffernet.h"

#define OBS_SIZE 4
#define NUM_ACTIONS 2
#ifndef HIDDEN_SIZE
#define HIDDEN_SIZE 8
#endif
#ifndef NUM_LAYERS
#define NUM_LAYERS 0
#endif

typedef struct {
    const char* name;
    int family;
    float a;
    float b;
    float lateral_offset_m;
    float heading_offset_rad;
} BenchCase;

typedef struct {
    int steps;
    float reward;
    float progress_m;
    float raw_action_sum[2];
    float wheel_cmd_sum[2];
    float speed_frac_sum;
    int idle_steps;
    int all_white_steps;
    int visible_steps;
} BenchStats;

static int expected_raw_float_count(void) {
    return HIDDEN_SIZE * OBS_SIZE
        + (NUM_ACTIONS + 1) * HIDDEN_SIZE
        + NUM_ACTIONS
        + NUM_LAYERS * 3 * HIDDEN_SIZE * HIDDEN_SIZE;
}

static void reset_recurrent_state(PufferNet* net) {
#if NUM_LAYERS > 0
    size_t count = (size_t)NUM_LAYERS * (size_t)net->num_agents * (size_t)HIDDEN_SIZE;
    memset(net->mingru->state, 0, count * sizeof(float));
#else
    (void)net;
#endif
}

static bool generate_case_track(LineFollow* env, const BenchCase* c) {
    if (c->family == LINE_FOLLOW_TRACK_STRAIGHT) {
        return generate_straight(&env->track, c->a, env->episode_line_width_m, env->track_bounds_m);
    }
    if (c->family == LINE_FOLLOW_TRACK_ARC) {
        return generate_arc(&env->track, c->a, c->b, env->episode_line_width_m, env->track_bounds_m);
    }
    if (c->family == LINE_FOLLOW_TRACK_S_CURVE) {
        return generate_s_curve(&env->track, c->a, c->b, env->episode_line_width_m, env->track_bounds_m);
    }
    if (c->family == LINE_FOLLOW_TRACK_OVAL) {
        return generate_oval(&env->track, c->a, c->b, env->episode_line_width_m, env->track_bounds_m);
    }
    return false;
}

static void reset_case(LineFollow* env, const BenchCase* c) {
    init(env);
    memset(&env->log, 0, sizeof(env->log));
    clear_terminal_info(env);

    env->episode_dt = env->dt;
    env->episode_line_width_m = env->line_width_m;
    env->episode_line_edge_softness_m = env->line_edge_softness_m;
    env->sensor_forward_jitter_m = 0.0f;
    env->sensor_lateral_jitter_m = 0.0f;
    env->qti_white_jitter = 0.0f;
    env->qti_black_jitter = 0.0f;
    env->line_width_jitter_m = 0.0f;
    env->line_edge_softness_jitter_m = 0.0f;
    env->sensor_noise_std = 0.0f;
    sensor_layout(env);

    if (!generate_case_track(env, c)) {
        fprintf(stderr, "failed to generate benchmark track: %s\n", c->name);
        exit(1);
    }

    const LineFollowTrackSample* start = &env->track.samples[0];
    float heading = atan2f(start->tangent_y, start->tangent_x) + c->heading_offset_rad;
    reset_runtime_pose(env,
        start->x + c->lateral_offset_m * start->normal_x,
        start->y + c->lateral_offset_m * start->normal_y,
        heading);

    env->last_progress = 0.0f;
    env->episode_progress = 0.0f;
    env->centerline_reward_progress_m = 0.0f;
    env->perf_quality_sum = 0.0f;
    env->perf_checkpoint_count = 0;
    env->centerline_error_sum = 0.0f;
    LineFollowNearest nearest = nearest_track(&env->track, env->robot_x, env->robot_y);
    env->centerline_error = nearest.signed_lateral_m;
    env->heading_error = angle_diff(env->robot_theta, nearest.heading_rad);
    compute_observations(env);
    record_trajectory(env);
}

static bool obs_all_white(const LineFollow* env) {
    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        if (env->observations[i] >= env->qti_threshold) {
            return false;
        }
    }
    return true;
}

static void accumulate_step_stats(LineFollow* env, BenchStats* stats) {
    float temp[2] = {env->actions[0], env->actions[1]};
    LineFollowWheelCommand cmd = map_actions(temp,
        env->max_wheel_speed_mps, env->command_deadband,
        env->left_speed_scale, env->right_speed_scale);
    stats->raw_action_sum[0] += env->actions[0];
    stats->raw_action_sum[1] += env->actions[1];
    stats->wheel_cmd_sum[0] += cmd.left_mps;
    stats->wheel_cmd_sum[1] += cmd.right_mps;
    float forward = 0.5f * (cmd.left_mps + cmd.right_mps);
    stats->speed_frac_sum += clampf(forward / fmaxf(env->max_wheel_speed_mps, 1e-6f), 0.0f, 1.0f);
    if (fmaxf(cmd.left_mps, cmd.right_mps) < env->min_wheel_action * env->max_wheel_speed_mps) {
        stats->idle_steps += 1;
    }
    if (obs_all_white(env)) {
        stats->all_white_steps += 1;
    }
    if (line_visible_to_sensor_array(env)) {
        stats->visible_steps += 1;
    }
}

static BenchStats run_case(LineFollow* env, PufferNet* net, const BenchCase* c) {
    BenchStats stats;
    memset(&stats, 0, sizeof(stats));
    reset_recurrent_state(net);
    reset_case(env, c);

    int max_steps = env->max_steps;
    for (int step = 0; step < max_steps; step++) {
        forward_puffernet(net, env->observations, env->actions);
        accumulate_step_stats(env, &stats);
        c_step(env);
        stats.steps += 1;
        stats.reward += env->last_reward;
        if (env->terminals[0] > 0.0f) {
            break;
        }
    }

    stats.progress_m = env->log.progress_m;
    return stats;
}

static void print_usage(const char* argv0) {
    fprintf(stderr,
        "Usage: %s WEIGHTS.bin [--dt SEC] [--dt-min SEC] [--dt-max SEC] "
        "[--motor-lag-alpha A] [--max-wheel-speed-mps MPS] [--max-steps N]\n",
        argv0);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 2;
    }

    const char* weights_path = argv[1];

    float observations[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[LINE_FOLLOW_NUM_ATNS] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};

    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    env.observations = observations;
    env.actions = actions;
    env.rewards = rewards;
    env.terminals = terminals;
    env.rng = 123u;

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--dt") == 0 && i + 1 < argc) {
            env.dt = strtof(argv[++i], NULL);
            env.dt_min = env.dt;
            env.dt_max = env.dt;
        } else if (strcmp(argv[i], "--dt-min") == 0 && i + 1 < argc) {
            env.dt_min = strtof(argv[++i], NULL);
        } else if (strcmp(argv[i], "--dt-max") == 0 && i + 1 < argc) {
            env.dt_max = strtof(argv[++i], NULL);
        } else if (strcmp(argv[i], "--motor-lag-alpha") == 0 && i + 1 < argc) {
            env.motor_lag_alpha = strtof(argv[++i], NULL);
        } else if (strcmp(argv[i], "--max-wheel-speed-mps") == 0 && i + 1 < argc) {
            env.max_wheel_speed_mps = strtof(argv[++i], NULL);
        } else if (strcmp(argv[i], "--max-steps") == 0 && i + 1 < argc) {
            env.max_steps = atoi(argv[++i]);
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    Weights* weights = load_weights(weights_path);
    if (weights == NULL) {
        return 1;
    }

    int raw_floats = weights->size - 7;
    int expected = expected_raw_float_count();
    if (raw_floats != expected) {
        fprintf(stderr,
            "Checkpoint size mismatch: raw_floats=%d expected=%d hidden_size=%d num_layers=%d\n",
            raw_floats, expected, HIDDEN_SIZE, NUM_LAYERS);
        free(weights);
        return 1;
    }

    int logit_sizes[NUM_ACTIONS] = {1, 1};
    PufferNet* net = make_puffernet(weights, 1, OBS_SIZE, HIDDEN_SIZE,
        NUM_LAYERS, logit_sizes, NUM_ACTIONS);

    const BenchCase cases[] = {
        {"straight_center", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, 0.000f, 0.000f},
        {"straight_inner_l", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, 0.020f, 0.000f},
        {"straight_inner_r", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, -0.020f, 0.000f},
        {"straight_outer_l", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, 0.040f, 0.000f},
        {"straight_outer_r", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, -0.040f, 0.000f},
        {"gentle_arc", LINE_FOLLOW_TRACK_ARC, 0.18f, 1.0f, 0.000f, 0.000f},
        {"s_curve", LINE_FOLLOW_TRACK_S_CURVE, 0.36f, 0.035f, 0.000f, 0.000f},
        {"oval", LINE_FOLLOW_TRACK_OVAL, 0.09f, 0.055f, 0.000f, 0.000f},
    };
    int num_cases = (int)(sizeof(cases) / sizeof(cases[0]));

    printf("weights=%s floats=%d hidden=%d layers=%d dt=%.3f motor_lag_alpha=%.3f max_wheel_speed_mps=%.3f\n",
        weights_path, raw_floats, HIDDEN_SIZE, NUM_LAYERS,
        env.dt, env.motor_lag_alpha, env.max_wheel_speed_mps);
    printf("case,steps,perf,progress_frac,progress_m,avg_raw_left,avg_raw_right,avg_cmd_left_mps,avg_cmd_right_mps,avg_speed_frac,idle_frac,all_white_frac,visible_frac,lost,timeout,complete,success,reward\n");

    float total_perf = 0.0f;
    float total_progress_frac = 0.0f;
    float total_idle_frac = 0.0f;
    float total_all_white_frac = 0.0f;
    for (int i = 0; i < num_cases; i++) {
        BenchStats stats = run_case(&env, net, &cases[i]);
        float steps = (float)fmaxf((float)stats.steps, 1.0f);
        total_perf += env.log.perf;
        total_progress_frac += env.log.progress_frac;
        total_idle_frac += (float)stats.idle_steps / steps;
        total_all_white_frac += (float)stats.all_white_steps / steps;
        printf("%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.0f,%.0f,%.0f,%.6f,%.6f\n",
            cases[i].name,
            stats.steps,
            env.log.perf,
            env.log.progress_frac,
            stats.progress_m,
            stats.raw_action_sum[0] / steps,
            stats.raw_action_sum[1] / steps,
            stats.wheel_cmd_sum[0] / steps,
            stats.wheel_cmd_sum[1] / steps,
            stats.speed_frac_sum / steps,
            (float)stats.idle_steps / steps,
            (float)stats.all_white_steps / steps,
            (float)stats.visible_steps / steps,
            env.log.terminal_lost_line,
            env.log.terminal_timeout,
            env.log.terminal_track_complete,
            env.log.terminal_success,
            stats.reward);
    }

    printf("summary,0,%.6f,%.6f,0,0,0,0,0,0,%.6f,%.6f,0,0,0,0,0,0\n",
        total_perf / (float)num_cases,
        total_progress_frac / (float)num_cases,
        total_idle_frac / (float)num_cases,
        total_all_white_frac / (float)num_cases);

    free_puffernet(net);
    free(weights);
    return 0;
}
