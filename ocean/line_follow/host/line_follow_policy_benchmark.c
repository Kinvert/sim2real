#define LINE_FOLLOW_NO_RENDER

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ocean/line_follow/line_follow.h"
#include "ocean/line_follow/host/line_follow_puffernet.h"

#define OBS_SIZE LINE_FOLLOW_OBS_SIZE
#define NUM_ACTIONS 2
#ifndef HIDDEN_SIZE
#define HIDDEN_SIZE 4
#endif
#ifndef NUM_LAYERS
#define NUM_LAYERS 0
#endif
#ifndef HEURISTIC_BASE
#define HEURISTIC_BASE 0.42f
#endif
#ifndef HEURISTIC_GAIN
#define HEURISTIC_GAIN 0.85f
#endif

typedef struct {
    const char* name;
    int family;
    float a;
    float b;
    float lateral_offset_m;
    float heading_offset_rad;
    bool mirror_y;
    unsigned int seed;
} BenchCase;

typedef enum {
    CONTROLLER_MODEL,
    CONTROLLER_STRAIGHT,
    CONTROLLER_HEURISTIC,
} Controller;

typedef enum {
    SUITE_STANDARD,
    SUITE_STRESS,
    SUITE_SEEDED_RANDOM,
    SUITE_ALL,
} BenchSuite;

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

typedef struct {
    const BenchCase* bench_case;
    BenchStats stats;
    float perf;
    float progress_frac;
    float effective_progress_m;
    float target_progress_m;
    float center_path_m;
    float path_efficiency;
    float avg_speed_frac;
    float idle_frac;
    float all_white_frac;
    float visible_frac;
    float lost;
    float timeout;
    float complete;
    float negative;
    float action_bound;
    float success;
} BenchResult;

static const BenchCase STANDARD_CASES[] = {
    {"straight_center", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, 0.000f, 0.000f, false, 0},
    {"straight_left_10mm", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, 0.010f, 0.000f, false, 0},
    {"straight_right_10mm", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, -0.010f, 0.000f, false, 0},
    {"straight_left_sensor", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, 0.018f, 0.000f, false, 0},
    {"straight_right_sensor", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, -0.018f, 0.000f, false, 0},
    {"straight_yaw_left", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, 0.000f, 0.20f, false, 0},
    {"straight_yaw_right", LINE_FOLLOW_TRACK_STRAIGHT, 0.36f, 0.0f, 0.000f, -0.20f, false, 0},
    {"arc_left", LINE_FOLLOW_TRACK_ARC, 0.18f, 1.0f, 0.000f, 0.000f, false, 0},
    {"arc_right", LINE_FOLLOW_TRACK_ARC, 0.18f, 1.0f, 0.000f, 0.000f, true, 0},
    {"s_curve", LINE_FOLLOW_TRACK_S_CURVE, 0.36f, 0.035f, 0.000f, 0.000f, false, 0},
    {"s_curve_mirror", LINE_FOLLOW_TRACK_S_CURVE, 0.36f, 0.035f, 0.000f, 0.000f, true, 0},
    {"oval", LINE_FOLLOW_TRACK_OVAL, 0.09f, 0.055f, 0.000f, 0.000f, false, 0},
};

static const BenchCase STRESS_CASES[] = {
    {"stress_left_25mm", LINE_FOLLOW_TRACK_STRAIGHT, 0.42f, 0.0f, 0.025f, 0.000f, false, 0},
    {"stress_right_25mm", LINE_FOLLOW_TRACK_STRAIGHT, 0.42f, 0.0f, -0.025f, 0.000f, false, 0},
    {"stress_yaw_left", LINE_FOLLOW_TRACK_STRAIGHT, 0.42f, 0.0f, 0.000f, 0.30f, false, 0},
    {"stress_yaw_right", LINE_FOLLOW_TRACK_STRAIGHT, 0.42f, 0.0f, 0.000f, -0.30f, false, 0},
    {"stress_arc_left", LINE_FOLLOW_TRACK_ARC, 0.12f, 1.25f, 0.000f, 0.000f, false, 0},
    {"stress_arc_right", LINE_FOLLOW_TRACK_ARC, 0.12f, 1.25f, 0.000f, 0.000f, true, 0},
    {"stress_s_curve", LINE_FOLLOW_TRACK_S_CURVE, 0.42f, 0.060f, 0.000f, 0.000f, false, 0},
    {"stress_s_curve_mirror", LINE_FOLLOW_TRACK_S_CURVE, 0.42f, 0.060f, 0.000f, 0.000f, true, 0},
    {"stress_oval", LINE_FOLLOW_TRACK_OVAL, 0.075f, 0.042f, 0.000f, 0.000f, false, 0},
};

static const BenchCase SEEDED_RANDOM_CASES[] = {
    {"seeded_random_1001", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, 0.000f, 0.000f, false, 1001u},
    {"seeded_random_1002", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, 0.010f, 0.050f, false, 1002u},
    {"seeded_random_1003", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, -0.010f, -0.050f, false, 1003u},
    {"seeded_random_1004", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, 0.018f, 0.000f, false, 1004u},
    {"seeded_random_1005", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, -0.018f, 0.000f, false, 1005u},
    {"seeded_random_1006", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, 0.000f, 0.150f, false, 1006u},
    {"seeded_random_1007", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, 0.000f, -0.150f, false, 1007u},
    {"seeded_random_1008", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, 0.012f, -0.100f, false, 1008u},
    {"seeded_random_1009", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, -0.012f, 0.100f, false, 1009u},
    {"seeded_random_1010", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, 0.000f, 0.000f, false, 1010u},
    {"seeded_random_1011", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, 0.010f, -0.150f, false, 1011u},
    {"seeded_random_1012", LINE_FOLLOW_TRACK_RANDOM, 0.0f, 0.0f, -0.010f, 0.150f, false, 1012u},
};

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
    bool ok = false;
    if (c->family == LINE_FOLLOW_TRACK_RANDOM) {
        unsigned int rng = c->seed != 0 ? c->seed : env->rng;
        ok = generate_track(&env->track, &rng, LINE_FOLLOW_TRACK_RANDOM,
            env->episode_line_width_m, env->track_bounds_m);
    } else if (c->family == LINE_FOLLOW_TRACK_STRAIGHT) {
        ok = generate_straight(&env->track, c->a, env->episode_line_width_m, env->track_bounds_m);
    } else if (c->family == LINE_FOLLOW_TRACK_ARC) {
        ok = generate_arc(&env->track, c->a, c->b, env->episode_line_width_m, env->track_bounds_m);
    } else if (c->family == LINE_FOLLOW_TRACK_S_CURVE) {
        ok = generate_s_curve(&env->track, c->a, c->b, env->episode_line_width_m, env->track_bounds_m);
    } else if (c->family == LINE_FOLLOW_TRACK_OVAL) {
        ok = generate_oval(&env->track, c->a, c->b, env->episode_line_width_m, env->track_bounds_m);
    }
    if (ok && c->mirror_y) {
        ok = track_mirror_y(&env->track);
    }
    return ok;
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
    env->line_width_segment_jitter_m = 0.0f;
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
    env->effective_progress_m = 0.0f;
    env->center_path_m = 0.0f;
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
        env->tire_diameter_m, env->min_drive_ticks_per_sec,
        env->left_speed_scale, env->right_speed_scale);
    stats->raw_action_sum[0] += env->actions[0];
    stats->raw_action_sum[1] += env->actions[1];
    stats->wheel_cmd_sum[0] += cmd.left_mps;
    stats->wheel_cmd_sum[1] += cmd.right_mps;
    float forward = 0.5f * (cmd.left_mps + cmd.right_mps);
    stats->speed_frac_sum += clampf(forward / fmaxf(env->max_wheel_speed_mps, 1e-6f), 0.0f, 1.0f);
    if (forward < env->min_wheel_action * env->max_wheel_speed_mps) {
        stats->idle_steps += 1;
    }
    if (obs_all_white(env)) {
        stats->all_white_steps += 1;
    }
    if (line_visible_to_sensor_array(env)) {
        stats->visible_steps += 1;
    }
}

static void set_heuristic_actions(LineFollow* env) {
    float left_dark = env->observations[0];
    float right_dark = env->observations[2];
    float correction = HEURISTIC_GAIN * (left_dark - right_dark);
    float base = HEURISTIC_BASE;

    env->actions[0] = raw_action_from_command(base - correction);
    env->actions[1] = raw_action_from_command(base + correction);
}

static BenchStats run_case(LineFollow* env, PufferNet* net, Controller controller,
        const BenchCase* c) {
    BenchStats stats;
    memset(&stats, 0, sizeof(stats));
    if (net != NULL) {
        reset_recurrent_state(net);
    }
    reset_case(env, c);

    int max_steps = env->max_steps;
    for (int step = 0; step < max_steps; step++) {
        if (controller == CONTROLLER_MODEL) {
            forward_puffernet(net, env->observations, env->actions);
        } else if (controller == CONTROLLER_HEURISTIC) {
            set_heuristic_actions(env);
        } else {
            env->actions[0] = raw_action_from_command(1.0f);
            env->actions[1] = raw_action_from_command(1.0f);
        }
        accumulate_step_stats(env, &stats);
        c_step(env);
        stats.steps += 1;
        stats.reward += env->rewards[0];
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
        "[--motor-lag-alpha A] [--max-wheel-speed-mps MPS] [--command-deadband X] "
        "[--min-drive-ticks-per-sec N] [--max-steps N] "
        "[--controller model|straight|heuristic] [--suite standard|stress|seeded_random|all] "
        "[--json PATH] [--training-log PATH]\n",
        argv0);
}

static const char* suite_name(BenchSuite suite) {
    if (suite == SUITE_STRESS) return "stress";
    if (suite == SUITE_SEEDED_RANDOM) return "seeded_random";
    if (suite == SUITE_ALL) return "all";
    return "standard";
}

static bool parse_suite(const char* value, BenchSuite* suite) {
    if (strcmp(value, "standard") == 0) {
        *suite = SUITE_STANDARD;
    } else if (strcmp(value, "stress") == 0) {
        *suite = SUITE_STRESS;
    } else if (strcmp(value, "seeded_random") == 0) {
        *suite = SUITE_SEEDED_RANDOM;
    } else if (strcmp(value, "all") == 0) {
        *suite = SUITE_ALL;
    } else {
        return false;
    }
    return true;
}

static void append_cases(const BenchCase** out, int* count,
        const BenchCase* cases, int num_cases) {
    for (int i = 0; i < num_cases; i++) {
        out[(*count)++] = &cases[i];
    }
}

static int select_cases(BenchSuite suite, const BenchCase** out) {
    int count = 0;
    if (suite == SUITE_STANDARD || suite == SUITE_ALL) {
        append_cases(out, &count, STANDARD_CASES,
            (int)(sizeof(STANDARD_CASES) / sizeof(STANDARD_CASES[0])));
    }
    if (suite == SUITE_STRESS || suite == SUITE_ALL) {
        append_cases(out, &count, STRESS_CASES,
            (int)(sizeof(STRESS_CASES) / sizeof(STRESS_CASES[0])));
    }
    if (suite == SUITE_SEEDED_RANDOM || suite == SUITE_ALL) {
        append_cases(out, &count, SEEDED_RANDOM_CASES,
            (int)(sizeof(SEEDED_RANDOM_CASES) / sizeof(SEEDED_RANDOM_CASES[0])));
    }
    return count;
}

static void write_json_summary(const char* path, const char* controller_name,
        const char* weights_path, BenchSuite suite, const LineFollow* env,
        int raw_floats, const char* training_log_path, const BenchResult* results,
        int num_results, float mean_perf, float mean_progress_frac,
        float mean_effective_progress_m, float mean_center_path_m,
        float mean_path_efficiency, float mean_speed_frac, float mean_idle_frac,
        float mean_all_white_frac) {
    if (path == NULL) {
        return;
    }
    FILE* f = fopen(path, "w");
    if (f == NULL) {
        fprintf(stderr, "failed to write json summary: %s\n", path);
        exit(1);
    }

    fprintf(f, "{\n");
    fprintf(f, "  \"controller\": \"%s\",\n", controller_name);
    fprintf(f, "  \"weights\": \"%s\",\n", weights_path);
    fprintf(f, "  \"checkpoint_path\": \"%s\",\n", weights_path);
    if (training_log_path != NULL) {
        fprintf(f, "  \"training_log_path\": \"%s\",\n", training_log_path);
    } else {
        fprintf(f, "  \"training_log_path\": null,\n");
    }
    fprintf(f, "  \"suite\": \"%s\",\n", suite_name(suite));
    fprintf(f, "  \"model\": {\n");
    fprintf(f, "    \"network\": \"MLP\",\n");
    fprintf(f, "    \"encoder\": \"DefaultEncoder\",\n");
    fprintf(f, "    \"decoder\": \"DefaultDecoder\",\n");
    fprintf(f, "    \"obs_size\": %d,\n", OBS_SIZE);
    fprintf(f, "    \"num_actions\": %d,\n", NUM_ACTIONS);
    fprintf(f, "    \"hidden_size\": %d,\n", HIDDEN_SIZE);
    fprintf(f, "    \"num_layers\": %d,\n", NUM_LAYERS);
    fprintf(f, "    \"raw_floats\": %d,\n", raw_floats);
    fprintf(f, "    \"raw_bytes\": %d\n", raw_floats * (int)sizeof(float));
    fprintf(f, "  },\n");
    fprintf(f, "  \"env\": {\n");
    fprintf(f, "    \"dt\": %.6f,\n", env->dt);
    fprintf(f, "    \"dt_min\": %.6f,\n", env->dt_min);
    fprintf(f, "    \"dt_max\": %.6f,\n", env->dt_max);
    fprintf(f, "    \"max_steps\": %d,\n", env->max_steps);
    fprintf(f, "    \"max_wheel_speed_mps\": %.6f,\n", env->max_wheel_speed_mps);
    fprintf(f, "    \"motor_lag_alpha\": %.6f,\n", env->motor_lag_alpha);
    fprintf(f, "    \"command_deadband\": %.6f,\n", env->command_deadband);
    fprintf(f, "    \"min_drive_ticks_per_sec\": %.6f,\n", env->min_drive_ticks_per_sec);
    fprintf(f, "    \"min_wheel_action\": %.6f,\n", env->min_wheel_action);
    fprintf(f, "    \"min_turn_outer_action\": %.6f,\n", env->min_turn_outer_action);
    fprintf(f, "    \"lost_line_limit\": %d,\n", env->lost_line_limit);
    fprintf(f, "    \"too_far_m\": %.6f,\n", env->too_far_m);
    fprintf(f, "    \"track_complete_margin_m\": %.6f\n", env->track_complete_margin_m);
    fprintf(f, "  },\n");
    fprintf(f, "  \"geometry\": {\n");
    fprintf(f, "    \"wheel_base_m\": %.6f,\n", env->wheel_base_m);
    fprintf(f, "    \"body_ahead_m\": %.6f,\n", env->body_ahead_m);
    fprintf(f, "    \"body_behind_m\": %.6f,\n", env->body_behind_m);
    fprintf(f, "    \"tire_diameter_m\": %.6f,\n", env->tire_diameter_m);
    fprintf(f, "    \"sensor_forward_m\": %.6f,\n", env->sensor_forward_m);
    fprintf(f, "    \"sensor_side_lateral_m\": %.6f,\n", env->sensor_side_lateral_m);
    fprintf(f, "    \"sensor_lateral_jitter_m\": %.6f,\n", env->sensor_lateral_jitter_m);
    fprintf(f, "    \"sensor_forward_jitter_m\": %.6f,\n", env->sensor_forward_jitter_m);
    fprintf(f, "    \"line_width_m\": %.6f,\n", env->line_width_m);
    fprintf(f, "    \"line_width_jitter_m\": %.6f,\n", env->line_width_jitter_m);
    fprintf(f, "    \"line_width_segment_jitter_m\": %.6f,\n", env->line_width_segment_jitter_m);
    fprintf(f, "    \"line_edge_softness_m\": %.6f,\n", env->line_edge_softness_m);
    fprintf(f, "    \"line_edge_softness_jitter_m\": %.6f\n", env->line_edge_softness_jitter_m);
    fprintf(f, "  },\n");
    fprintf(f, "  \"qti\": {\n");
    fprintf(f, "    \"white_time\": %.6f,\n", env->qti_white_time);
    fprintf(f, "    \"black_time\": %.6f,\n", env->qti_black_time);
    fprintf(f, "    \"white_jitter\": %.6f,\n", env->qti_white_jitter);
    fprintf(f, "    \"black_jitter\": %.6f,\n", env->qti_black_jitter);
    fprintf(f, "    \"noise_std\": %.6f,\n", env->sensor_noise_std);
    fprintf(f, "    \"threshold\": %.6f,\n", env->qti_threshold);
    fprintf(f, "    \"timeout\": %.6f\n", env->qti_timeout);
    fprintf(f, "  },\n");
    fprintf(f, "  \"reward\": {\n");
    fprintf(f, "    \"progress_reward_scale\": %.6f,\n", env->progress_reward_scale);
    fprintf(f, "    \"centerline_penalty_scale\": %.6f,\n", env->centerline_penalty_scale);
    fprintf(f, "    \"centerline_reward_interval_m\": %.6f,\n", env->centerline_reward_interval_m);
    fprintf(f, "    \"success_reward\": %.6f,\n", env->success_reward);
    fprintf(f, "    \"heading_penalty_scale\": %.6f,\n", env->heading_penalty_scale);
    fprintf(f, "    \"lost_line_penalty\": %.6f,\n", env->lost_line_penalty);
    fprintf(f, "    \"off_track_terminal_penalty\": %.6f,\n", env->off_track_terminal_penalty);
    fprintf(f, "    \"action_smoothness_penalty\": %.6f,\n", env->action_smoothness_penalty);
    fprintf(f, "    \"action_bound_penalty_scale\": %.6f,\n", env->action_bound_penalty_scale);
    fprintf(f, "    \"turn_penalty_scale\": %.6f,\n", env->turn_penalty_scale);
    fprintf(f, "    \"steering_correction_scale\": %.6f,\n", env->steering_correction_scale);
    fprintf(f, "    \"turn_speed_penalty_scale\": %.6f,\n", env->turn_speed_penalty_scale);
    fprintf(f, "    \"time_penalty\": %.6f,\n", env->time_penalty);
    fprintf(f, "    \"idle_penalty\": %.6f,\n", env->idle_penalty);
    fprintf(f, "    \"reverse_penalty_scale\": %.6f\n", env->reverse_penalty_scale);
    fprintf(f, "  },\n");
    fprintf(f, "  \"summary\": {\n");
    fprintf(f, "    \"perf\": %.6f,\n", mean_perf);
    fprintf(f, "    \"progress_frac\": %.6f,\n", mean_progress_frac);
    fprintf(f, "    \"effective_progress_m\": %.6f,\n", mean_effective_progress_m);
    fprintf(f, "    \"center_path_m\": %.6f,\n", mean_center_path_m);
    fprintf(f, "    \"path_efficiency\": %.6f,\n", mean_path_efficiency);
    fprintf(f, "    \"avg_speed_frac\": %.6f,\n", mean_speed_frac);
    fprintf(f, "    \"idle_frac\": %.6f,\n", mean_idle_frac);
    fprintf(f, "    \"all_white_frac\": %.6f\n", mean_all_white_frac);
    fprintf(f, "  },\n");
    fprintf(f, "  \"cases\": [\n");
    for (int i = 0; i < num_results; i++) {
        const BenchResult* r = &results[i];
        fprintf(f,
            "    {\"name\":\"%s\",\"steps\":%d,\"perf\":%.6f,"
            "\"progress_frac\":%.6f,\"track_progress_m\":%.6f,"
            "\"effective_progress_m\":%.6f,\"target_progress_m\":%.6f,"
            "\"center_path_m\":%.6f,\"path_efficiency\":%.6f,"
            "\"avg_speed_frac\":%.6f,\"idle_frac\":%.6f,"
            "\"all_white_frac\":%.6f,\"visible_frac\":%.6f,"
            "\"lost\":%.0f,\"timeout\":%.0f,\"complete\":%.0f,"
            "\"negative\":%.0f,\"action_bound\":%.0f,\"success\":%.6f,"
            "\"reward\":%.6f}%s\n",
            r->bench_case->name,
            r->stats.steps,
            r->perf,
            r->progress_frac,
            r->stats.progress_m,
            r->effective_progress_m,
            r->target_progress_m,
            r->center_path_m,
            r->path_efficiency,
            r->avg_speed_frac,
            r->idle_frac,
            r->all_white_frac,
            r->visible_frac,
            r->lost,
            r->timeout,
            r->complete,
            r->negative,
            r->action_bound,
            r->success,
            r->stats.reward,
            i + 1 == num_results ? "" : ",");
    }
    fprintf(f, "  ]\n");
    fprintf(f, "}\n");
    fclose(f);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 2;
    }

    const char* weights_path = argv[1];
    Controller controller = CONTROLLER_MODEL;
    BenchSuite suite = SUITE_STANDARD;
    const char* json_path = NULL;
    const char* training_log_path = NULL;

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

    bool explicit_timing = false;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--dt") == 0 && i + 1 < argc) {
            env.dt = strtof(argv[++i], NULL);
            env.dt_min = env.dt;
            env.dt_max = env.dt;
            explicit_timing = true;
        } else if (strcmp(argv[i], "--dt-min") == 0 && i + 1 < argc) {
            env.dt_min = strtof(argv[++i], NULL);
            explicit_timing = true;
        } else if (strcmp(argv[i], "--dt-max") == 0 && i + 1 < argc) {
            env.dt_max = strtof(argv[++i], NULL);
            explicit_timing = true;
        } else if (strcmp(argv[i], "--motor-lag-alpha") == 0 && i + 1 < argc) {
            env.motor_lag_alpha = strtof(argv[++i], NULL);
        } else if (strcmp(argv[i], "--max-wheel-speed-mps") == 0 && i + 1 < argc) {
            env.max_wheel_speed_mps = strtof(argv[++i], NULL);
        } else if (strcmp(argv[i], "--command-deadband") == 0 && i + 1 < argc) {
            env.command_deadband = strtof(argv[++i], NULL);
        } else if (strcmp(argv[i], "--min-drive-ticks-per-sec") == 0 && i + 1 < argc) {
            env.min_drive_ticks_per_sec = strtof(argv[++i], NULL);
        } else if (strcmp(argv[i], "--max-steps") == 0 && i + 1 < argc) {
            env.max_steps = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--controller") == 0 && i + 1 < argc) {
            const char* value = argv[++i];
            if (strcmp(value, "model") == 0) {
                controller = CONTROLLER_MODEL;
            } else if (strcmp(value, "straight") == 0) {
                controller = CONTROLLER_STRAIGHT;
            } else if (strcmp(value, "heuristic") == 0) {
                controller = CONTROLLER_HEURISTIC;
            } else {
                print_usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--suite") == 0 && i + 1 < argc) {
            if (!parse_suite(argv[++i], &suite)) {
                print_usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--json") == 0 && i + 1 < argc) {
            json_path = argv[++i];
        } else if (strcmp(argv[i], "--training-log") == 0 && i + 1 < argc) {
            training_log_path = argv[++i];
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }
    if (!explicit_timing) {
        apply_model_timing(&env);
    }

    Weights* weights = NULL;
    PufferNet* net = NULL;
    int raw_floats = 0;
    if (controller == CONTROLLER_MODEL) {
        weights = load_weights(weights_path);
        if (weights == NULL) {
            return 1;
        }

        raw_floats = weights->size - 7;
        int expected = expected_raw_float_count();
        if (raw_floats != expected) {
            fprintf(stderr,
                "Checkpoint size mismatch: raw_floats=%d expected=%d hidden_size=%d num_layers=%d\n",
                raw_floats, expected, HIDDEN_SIZE, NUM_LAYERS);
            free(weights);
            return 1;
        }

        int logit_sizes[NUM_ACTIONS] = {1, 1};
        net = make_puffernet(weights, 1, OBS_SIZE, HIDDEN_SIZE,
            NUM_LAYERS, logit_sizes, NUM_ACTIONS);
    }

    const BenchCase* cases[64];
    int num_cases = select_cases(suite, cases);
    BenchResult results[64];
    memset(results, 0, sizeof(results));

    const char* controller_name = controller == CONTROLLER_MODEL ? "model"
        : controller == CONTROLLER_HEURISTIC ? "heuristic" : "straight";
    printf("controller=%s checkpoint_path=%s training_log_path=%s suite=%s floats=%d hidden=%d layers=%d dt=%.3f motor_lag_alpha=%.3f max_wheel_speed_mps=%.3f\n",
        controller_name, weights_path,
        training_log_path != NULL ? training_log_path : "none",
        suite_name(suite), raw_floats,
        HIDDEN_SIZE, NUM_LAYERS, env.dt, env.motor_lag_alpha,
        env.max_wheel_speed_mps);
    printf("case,steps,perf,progress_frac,track_progress_m,effective_progress_m,target_progress_m,center_path_m,path_efficiency,avg_raw_left,avg_raw_right,avg_cmd_left_mps,avg_cmd_right_mps,avg_speed_frac,idle_frac,all_white_frac,visible_frac,lost,timeout,complete,negative,action_bound,success,reward\n");

    float total_perf = 0.0f;
    float total_progress_frac = 0.0f;
    float total_effective_progress_m = 0.0f;
    float total_center_path_m = 0.0f;
    float total_path_efficiency = 0.0f;
    float total_speed_frac = 0.0f;
    float total_idle_frac = 0.0f;
    float total_all_white_frac = 0.0f;
    for (int i = 0; i < num_cases; i++) {
        BenchStats stats = run_case(&env, net, controller, cases[i]);
        float steps = (float)fmaxf((float)stats.steps, 1.0f);
        BenchResult* result = &results[i];
        result->bench_case = cases[i];
        result->stats = stats;
        result->perf = env.log.perf;
        result->progress_frac = env.log.progress_frac;
        result->effective_progress_m = env.log.effective_progress_m;
        result->target_progress_m = env.log.progress_target_m;
        result->center_path_m = env.log.center_path_m;
        result->path_efficiency = env.log.path_efficiency;
        result->avg_speed_frac = stats.speed_frac_sum / steps;
        result->idle_frac = (float)stats.idle_steps / steps;
        result->all_white_frac = (float)stats.all_white_steps / steps;
        result->visible_frac = (float)stats.visible_steps / steps;
        result->lost = env.log.terminal_lost_line;
        result->timeout = env.log.terminal_timeout;
        result->complete = env.log.terminal_track_complete;
        result->negative = env.log.terminal_negative;
        result->action_bound = env.log.terminal_action_bound;
        result->success = env.log.terminal_success;

        total_perf += env.log.perf;
        total_progress_frac += env.log.progress_frac;
        total_effective_progress_m += result->effective_progress_m;
        total_center_path_m += result->center_path_m;
        total_path_efficiency += result->path_efficiency;
        total_speed_frac += result->avg_speed_frac;
        total_idle_frac += result->idle_frac;
        total_all_white_frac += result->all_white_frac;
        printf("%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.0f,%.0f,%.0f,%.0f,%.0f,%.6f,%.6f\n",
            cases[i]->name,
            stats.steps,
            result->perf,
            result->progress_frac,
            stats.progress_m,
            result->effective_progress_m,
            result->target_progress_m,
            result->center_path_m,
            result->path_efficiency,
            stats.raw_action_sum[0] / steps,
            stats.raw_action_sum[1] / steps,
            stats.wheel_cmd_sum[0] / steps,
            stats.wheel_cmd_sum[1] / steps,
            result->avg_speed_frac,
            result->idle_frac,
            result->all_white_frac,
            result->visible_frac,
            result->lost,
            result->timeout,
            result->complete,
            result->negative,
            result->action_bound,
            result->success,
            stats.reward);
    }

    float denom = (float)num_cases;
    float mean_perf = total_perf / denom;
    float mean_progress_frac = total_progress_frac / denom;
    float mean_effective_progress_m = total_effective_progress_m / denom;
    float mean_center_path_m = total_center_path_m / denom;
    float mean_path_efficiency = total_path_efficiency / denom;
    float mean_speed_frac = total_speed_frac / denom;
    float mean_idle_frac = total_idle_frac / denom;
    float mean_all_white_frac = total_all_white_frac / denom;
    printf("summary,0,%.6f,%.6f,0,%.6f,0,%.6f,%.6f,0,0,0,0,%.6f,%.6f,%.6f,0,0,0,0,0,0,0,0\n",
        mean_perf,
        mean_progress_frac,
        mean_effective_progress_m,
        mean_center_path_m,
        mean_path_efficiency,
        mean_speed_frac,
        mean_idle_frac,
        mean_all_white_frac);

    write_json_summary(json_path, controller_name, weights_path, suite, &env,
        raw_floats, training_log_path, results, num_cases,
        mean_perf, mean_progress_frac, mean_effective_progress_m,
        mean_center_path_m, mean_path_efficiency, mean_speed_frac,
        mean_idle_frac, mean_all_white_frac);

    if (net != NULL) {
        free_puffernet(net);
    }
    free(weights);
    return 0;
}
