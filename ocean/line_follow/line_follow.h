#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef LINE_FOLLOW_NO_RENDER
#include "raylib.h"
#endif

#include "line_follow_track.h"

#define LINE_FOLLOW_SENSOR_COUNT 3
#define LINE_FOLLOW_OBS_SIZE 3
#define LINE_FOLLOW_NUM_ATNS 2
#define LINE_FOLLOW_TRAJECTORY_CAP 512
#define LINE_FOLLOW_ACCURACY_ZERO_M 0.030f
#define LINE_FOLLOW_REWARD_ZERO_M 0.015f
#define LINE_FOLLOW_TICKS_PER_REV 64.0f
#define LINE_FOLLOW_LEFT 0
#define LINE_FOLLOW_MIDDLE 1
#define LINE_FOLLOW_RIGHT 2

typedef struct Log Log;
struct Log {
    float perf;
    float score;
    float progress_frac;
    float distance_progress_frac;
    float progress_m;
    float effective_progress_m;
    float center_path_m;
    float path_efficiency;
    float episode_return;
    float episode_length;
    float centerline_error;
    float accuracy;
    float checkpoint_accuracy;
    float checkpoints;
    float target_checkpoints;
    float progress_target_m;
    float avg_forward_speed_mps;
    float avg_speed_frac;
    float idle_frac;
    float turn_outer_speed_frac;
    float negative_action_frac;
    float action_bound_violation;
    float raw_action_abs;
    float terminal_timeout;
    float terminal_lost_line;
    float terminal_too_far;
    float terminal_track_complete;
    float terminal_negative;
    float terminal_action_bound;
    float terminal_success;
    float n;
};

typedef struct Client Client;
struct Client {
    int unused;
};

typedef struct LineFollowWheelCommand LineFollowWheelCommand;
struct LineFollowWheelCommand {
    float left_action;
    float right_action;
    float left_mps;
    float right_mps;
};

typedef struct LineFollow LineFollow;
struct LineFollow {
    Log log;
    float* observations;
    float* actions;
    float* rewards;
    float* terminals;
    int num_agents;
    unsigned int rng;

    Client* client;
    LineFollowTrack track;

    int tick;
    int max_steps;
    int lost_line_steps;
    int idle_steps;
    int negative_action_steps;
    int lost_line_limit;
    int track_family;
    int max_track_gen_attempts;
    int policy_hidden_size;
    int policy_num_layers;
    int model_dt_enabled;

    float dt;
    float dt_min;
    float dt_max;
    float episode_dt;
    float max_wheel_speed_mps;
    float wheel_base_m;
    float body_ahead_m;
    float body_behind_m;
    float body_width_m;
    float tire_diameter_m;
    float tire_width_m;
    float motor_lag_alpha;
    float command_deadband;
    float min_drive_ticks_per_sec;
    float left_speed_scale;
    float right_speed_scale;

    float robot_x;
    float robot_y;
    float robot_theta; // theta=0 means facing along the positive x-axis, increasing counterclockwise
    float v_left;
    float v_right;
    float v_left_cmd;
    float v_right_cmd;
    float prev_action[2];

    float episode_return;
    float episode_progress;
    float effective_progress_m;
    float center_path_m;
    float last_progress;
    float centerline_reward_progress_m;
    float perf_quality_sum;
    int perf_checkpoint_count;
    float forward_speed_sum;
    float speed_frac_sum;
    float turn_outer_speed_frac_sum;
    float action_bound_violation_sum;
    float raw_action_abs_sum;
    float centerline_error;
    float centerline_error_sum;
    float heading_error;
    float last_reward;
    int turn_step_count;
    float terminal_timeout;
    float terminal_lost_line;
    float terminal_too_far;
    float terminal_track_complete;
    float terminal_negative;
    float terminal_action_bound;
    float terminal_success;

    float progress_reward_scale;
    float centerline_penalty_scale;
    float centerline_reward_interval_m;
    float success_reward;
    float heading_penalty_scale;
    float lost_line_penalty;
    float off_track_terminal_penalty;
    float action_smoothness_penalty;
    float action_bound_penalty_scale;
    float turn_penalty_scale;
    float steering_correction_scale;
    float turn_speed_penalty_scale;
    float min_turn_outer_action;
    float time_penalty;
    float idle_penalty;
    float min_wheel_action;
    float reverse_penalty_scale;
    float too_far_m;
    float track_complete_margin_m;

    float sensor_forward_m;
    float sensor_side_lateral_m;
    float sensor_lateral_jitter_m;
    float sensor_forward_jitter_m;
    float sensor_noise_std;
    float sensor_gain[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_bias[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_white_time[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_black_time[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_forward[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_lateral[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_x[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_y[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_raw[LINE_FOLLOW_SENSOR_COUNT];
    float sensor_obs[LINE_FOLLOW_SENSOR_COUNT];
    unsigned char sensor_bits[LINE_FOLLOW_SENSOR_COUNT];
    float qti_threshold;

    float qti_white_time;
    float qti_black_time;
    float qti_white_jitter;
    float qti_black_jitter;
    float qti_timeout;
    float line_width_m;
    float line_width_jitter_m;
    float line_width_segment_jitter_m;
    float line_edge_softness_m;
    float line_edge_softness_jitter_m;
    float episode_line_width_m;
    float episode_line_edge_softness_m;
    float track_bounds_m;
    float start_lateral_offset_m;
    float start_heading_offset_rad;

    float trajectory_x[LINE_FOLLOW_TRAJECTORY_CAP];
    float trajectory_y[LINE_FOLLOW_TRAJECTORY_CAP];
    int trajectory_count;
};

static inline float clampf(float value, float lo, float hi) {
    if (!isfinite(value)) return 0.0f;
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static inline float rand_signed(unsigned int* rng) {
    return 2.0f * ((float)rand_r(rng) / (float)RAND_MAX) - 1.0f;
}

void set_defaults(LineFollow* env) {
    env->num_agents = 1;
    env->max_steps = 800;
    env->lost_line_limit = 16;
    env->track_family = LINE_FOLLOW_TRACK_RANDOM;
    env->max_track_gen_attempts = 4;
    env->policy_hidden_size = 8;
    env->policy_num_layers = 0;
    env->model_dt_enabled = 1;

    env->dt = 0.024f;
    env->dt_min = 0.020f;
    env->dt_max = 0.035f;
    env->episode_dt = env->dt;
    env->max_wheel_speed_mps = 0.116f;
    env->wheel_base_m = 0.125f;
    env->body_ahead_m = 0.040f;
    env->body_behind_m = 0.090f;
    env->body_width_m = 0.075f;
    env->tire_diameter_m = 0.065f;
    env->tire_width_m = 0.012f;
    env->motor_lag_alpha = 0.65f;
    env->command_deadband = 0.04f;
    env->min_drive_ticks_per_sec = 6.0f;
    env->left_speed_scale = 1.0f;
    env->right_speed_scale = 1.0f;

    env->progress_reward_scale = 1.0f;
    env->centerline_penalty_scale = 0.0f;
    env->centerline_reward_interval_m = 0.010f;
    env->success_reward = 1.0f;
    env->heading_penalty_scale = 0.0f;
    env->lost_line_penalty = 0.03f;
    env->off_track_terminal_penalty = 1.0f;
    env->action_smoothness_penalty = 0.0f;
    env->action_bound_penalty_scale = 0.01f;
    env->turn_penalty_scale = 0.0f;
    env->steering_correction_scale = 0.30f;
    env->turn_speed_penalty_scale = 0.05f;
    env->min_turn_outer_action = 0.65f;
    env->time_penalty = 0.005f;
    env->idle_penalty = 0.02f;
    env->min_wheel_action = 0.35f;
    env->reverse_penalty_scale = 0.2f;
    env->too_far_m = 0.09f;
    env->track_complete_margin_m = 0.02f;

    env->sensor_forward_m = 0.0435f;
    env->sensor_side_lateral_m = 0.018f;
    env->sensor_lateral_jitter_m = 0.004f;
    env->sensor_forward_jitter_m = 0.003f;
    env->sensor_noise_std = 0.0f;
    env->qti_threshold = 0.5f;
    env->qti_white_time = 40.0f;
    env->qti_black_time = 350.0f;
    env->qti_white_jitter = 25.0f;
    env->qti_black_jitter = 120.0f;
    env->qti_timeout = 2500.0f;
    env->line_width_m = 0.018f;
    env->line_width_jitter_m = 0.006f;
    env->line_width_segment_jitter_m = 0.002f;
    env->line_edge_softness_m = 0.006f;
    env->line_edge_softness_jitter_m = 0.004f;
    env->episode_line_width_m = env->line_width_m;
    env->episode_line_edge_softness_m = env->line_edge_softness_m;
    env->track_bounds_m = 1.0f;
    env->start_lateral_offset_m = 0.025f;
    env->start_heading_offset_rad = 0.261799f;

    const float lateral[LINE_FOLLOW_SENSOR_COUNT] = {
        env->sensor_side_lateral_m,
        0.0f,
        -env->sensor_side_lateral_m,
    };
    for (int i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++) {
        env->sensor_gain[i] = 1.0f;
        env->sensor_bias[i] = 0.0f;
        env->sensor_white_time[i] = env->qti_white_time;
        env->sensor_black_time[i] = env->qti_black_time;
        env->sensor_forward[i] = env->sensor_forward_m;
        env->sensor_lateral[i] = lateral[i];
        env->sensor_obs[i] = 0.0f;
    }
}

void init(LineFollow* env) {
    memset(&env->log, 0, sizeof(Log));
    env->tick = 0;
    env->lost_line_steps = 0;
    env->idle_steps = 0;
    env->negative_action_steps = 0;
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
    env->turn_outer_speed_frac_sum = 0.0f;
    env->action_bound_violation_sum = 0.0f;
    env->raw_action_abs_sum = 0.0f;
    env->centerline_error = 0.0f;
    env->centerline_error_sum = 0.0f;
    env->heading_error = 0.0f;
    env->last_reward = 0.0f;
    env->turn_step_count = 0;
    env->terminal_timeout = 0.0f;
    env->terminal_lost_line = 0.0f;
    env->terminal_too_far = 0.0f;
    env->terminal_track_complete = 0.0f;
    env->terminal_negative = 0.0f;
    env->terminal_action_bound = 0.0f;
    env->terminal_success = 0.0f;
    env->episode_dt = env->dt;
    env->v_left = 0.0f;
    env->v_right = 0.0f;
    env->v_left_cmd = 0.0f;
    env->v_right_cmd = 0.0f;
    env->prev_action[0] = 0.0f;
    env->prev_action[1] = 0.0f;
    env->trajectory_count = 0;
}

void allocate(LineFollow* env) {
    set_defaults(env);
    init(env);
    env->observations = (float*)calloc(LINE_FOLLOW_OBS_SIZE, sizeof(float));
    env->actions = (float*)calloc(LINE_FOLLOW_NUM_ATNS, sizeof(float));
    env->rewards = (float*)calloc(1, sizeof(float));
    env->terminals = (float*)calloc(1, sizeof(float));
}

void free_allocated(LineFollow* env) {
    free(env->observations);
    free(env->actions);
    free(env->rewards);
    free(env->terminals);
}

static inline float accuracy_score(LineFollow* env, float centerline_error) {
    (void)env;
    float error = fabsf(centerline_error);
    return clampf(1.0f - error / LINE_FOLLOW_ACCURACY_ZERO_M, 0.0f, 1.0f);
}

static inline float reward_centerline_score(LineFollow* env, float centerline_error) {
    (void)env;
    float error = fabsf(centerline_error);
    return clampf(1.0f - error / LINE_FOLLOW_REWARD_ZERO_M, -1.0f, 1.0f);
}

static inline float progress_fraction_score(LineFollow* env, float progress_delta) {
    float dt = env->episode_dt > 0.0f ? env->episode_dt : env->dt;
    float max_tick_progress = fmaxf(env->max_wheel_speed_mps * dt, 1e-6f);
    return clampf(progress_delta / max_tick_progress, -1.0f, 1.0f);
}

static inline void clear_terminal_info(LineFollow* env) {
    env->terminal_timeout = 0.0f;
    env->terminal_lost_line = 0.0f;
    env->terminal_too_far = 0.0f;
    env->terminal_track_complete = 0.0f;
    env->terminal_negative = 0.0f;
    env->terminal_action_bound = 0.0f;
    env->terminal_success = 0.0f;
}

static inline float perf_interval_m(LineFollow* env) {
    return env->centerline_reward_interval_m > 1e-6f
        ? env->centerline_reward_interval_m
        : 0.025f;
}

static inline float checkpoint_reward_scale(LineFollow* env) {
    float interval = perf_interval_m(env);
    return interval / 0.025f;
}

static inline float perf_target_progress_m(LineFollow* env) {
    float dt = env->episode_dt > 0.0f ? env->episode_dt : env->dt;
    float max_physical_progress = (float)env->max_steps
        * fmaxf(env->max_wheel_speed_mps, 0.0f) * fmaxf(dt, 0.0f);
    float target = env->track.length_m > 1e-6f ? env->track.length_m : max_physical_progress;
    if (max_physical_progress > 1e-6f) {
        target = fminf(target, max_physical_progress);
    }
    return target;
}

static inline int perf_target_checkpoints(LineFollow* env) {
    float interval = perf_interval_m(env);
    float target = perf_target_progress_m(env);
    int checkpoints = (int)floorf(target / interval);
    return checkpoints < 1 ? 1 : checkpoints;
}

static inline float episode_progress_frac(LineFollow* env) {
    float target = perf_target_progress_m(env);
    return target > 1e-6f
        ? clampf(env->episode_progress / target, 0.0f, 1.0f)
        : 0.0f;
}

static inline float episode_checkpoint_accuracy(LineFollow* env) {
    return env->perf_checkpoint_count > 0
        ? clampf(env->perf_quality_sum / (float)env->perf_checkpoint_count, 0.0f, 1.0f)
        : 0.0f;
}

static inline float episode_perf_score(LineFollow* env) {
    float target = perf_target_progress_m(env);
    return target > 1e-6f
        ? clampf(env->effective_progress_m / target, 0.0f, 1.0f)
        : 0.0f;
}

void add_log(LineFollow* env) {
    float episode_length = (float)env->tick;
    float mean_error = episode_length > 0.0f ? env->centerline_error_sum / episode_length : 0.0f;
    float target_progress = perf_target_progress_m(env);
    float distance_progress_frac = target_progress > 1e-6f
        ? clampf(env->episode_progress / target_progress, 0.0f, 1.0f)
        : 0.0f;
    float current_accuracy = accuracy_score(env, env->centerline_error);
    int target_checkpoints = perf_target_checkpoints(env);
    float progress_frac = episode_progress_frac(env);
    float checkpoint_accuracy = episode_checkpoint_accuracy(env);
    float perf = episode_perf_score(env);
    float path_efficiency = env->center_path_m > 1e-6f
        ? clampf(env->episode_progress / env->center_path_m, 0.0f, 1.0f)
        : 0.0f;
    float avg_forward_speed = episode_length > 0.0f
        ? env->forward_speed_sum / episode_length
        : 0.0f;
    float avg_speed_frac = episode_length > 0.0f
        ? env->speed_frac_sum / episode_length
        : 0.0f;
    float idle_frac = episode_length > 0.0f
        ? (float)env->idle_steps / episode_length
        : 0.0f;
    float turn_outer_speed_frac = env->turn_step_count > 0
        ? env->turn_outer_speed_frac_sum / (float)env->turn_step_count
        : 0.0f;
    float negative_action_frac = episode_length > 0.0f
        ? (float)env->negative_action_steps / episode_length
        : 0.0f;
    float action_bound_violation = episode_length > 0.0f
        ? env->action_bound_violation_sum / episode_length
        : 0.0f;
    float raw_action_abs = episode_length > 0.0f
        ? env->raw_action_abs_sum / episode_length
        : 0.0f;

    env->log.perf += perf;
    env->log.score += env->episode_return;
    env->log.progress_frac += progress_frac;
    env->log.distance_progress_frac += distance_progress_frac;
    env->log.progress_m += env->episode_progress;
    env->log.effective_progress_m += env->effective_progress_m;
    env->log.center_path_m += env->center_path_m;
    env->log.path_efficiency += path_efficiency;
    env->log.episode_return += env->episode_return;
    env->log.episode_length += episode_length;
    env->log.centerline_error += mean_error;
    env->log.accuracy += current_accuracy;
    env->log.checkpoint_accuracy += checkpoint_accuracy;
    env->log.checkpoints += (float)env->perf_checkpoint_count;
    env->log.target_checkpoints += (float)target_checkpoints;
    env->log.progress_target_m += target_progress;
    env->log.avg_forward_speed_mps += avg_forward_speed;
    env->log.avg_speed_frac += avg_speed_frac;
    env->log.idle_frac += idle_frac;
    env->log.turn_outer_speed_frac += turn_outer_speed_frac;
    env->log.negative_action_frac += negative_action_frac;
    env->log.action_bound_violation += action_bound_violation;
    env->log.raw_action_abs += raw_action_abs;
    env->log.terminal_timeout += env->terminal_timeout;
    env->log.terminal_lost_line += env->terminal_lost_line;
    env->log.terminal_too_far += env->terminal_too_far;
    env->log.terminal_track_complete += env->terminal_track_complete;
    env->log.terminal_negative += env->terminal_negative;
    env->log.terminal_action_bound += env->terminal_action_bound;
    env->log.terminal_success += env->terminal_success;
    env->log.n += 1.0f;
}

float qti_normalize(float raw, float white_time, float black_time) {
    float denom = black_time - white_time;
    if (fabsf(denom) < 1e-6f) {
        return raw >= black_time ? 1.0f : 0.0f;
    }
    return clampf((raw - white_time) / denom, 0.0f, 1.0f);
}

static inline float jittered_positive(float base, float jitter, float floor, unsigned int* rng) {
    return fmaxf(floor, base + jitter * rand_signed(rng));
}

static inline float sample_episode_dt(LineFollow* env) {
    float nominal = env->dt > 0.0f ? env->dt : 0.025f;
    float lo = env->dt_min > 0.0f ? env->dt_min : nominal;
    float hi = env->dt_max > 0.0f ? env->dt_max : nominal;
    if (lo > hi) {
        float tmp = lo;
        lo = hi;
        hi = tmp;
    }
    lo = fmaxf(lo, 0.001f);
    hi = fmaxf(hi, lo);
    float unit = (float)rand_r(&env->rng) / (float)RAND_MAX;
    return lo + unit * (hi - lo);
}

static inline float estimate_policy_dt(int hidden_size, int num_layers) {
    int h = hidden_size > 0 ? hidden_size : 1;
    int layers = num_layers > 0 ? num_layers : 0;

    if (layers == 0) {
        if (h <= 2) return 0.018f;
        if (h <= 4) return 0.020f;
        if (h <= 8) return 0.024f;
        return 0.024f + 0.003f * (float)(h - 8);
    }

    if (h <= 2) return 0.070f;
    if (h <= 4) return 0.105f;
    if (h <= 8) return 0.188f;
    if (h <= 16) {
        return 0.188f + (0.477f - 0.188f) * (float)(h - 8) / 8.0f;
    }
    return 0.477f + 0.040f * (float)(h - 16);
}

static inline int scale_step_limit_for_dt(int steps, float reference_dt, float dt) {
    if (steps <= 1 || reference_dt <= 0.0f || dt <= 0.0f) {
        return steps;
    }
    int scaled = (int)ceilf((float)steps * reference_dt / dt);
    return scaled < 1 ? 1 : scaled;
}

void apply_model_timing(LineFollow* env) {
    if (!env->model_dt_enabled) {
        return;
    }

    float reference_dt = env->dt > 0.0f ? env->dt : 0.065f;
    float min_ratio = env->dt_min > 0.0f ? env->dt_min / reference_dt : 1.0f;
    float max_ratio = env->dt_max > 0.0f ? env->dt_max / reference_dt : 1.0f;
    if (min_ratio > max_ratio) {
        float tmp = min_ratio;
        min_ratio = max_ratio;
        max_ratio = tmp;
    }

    float dt = estimate_policy_dt(env->policy_hidden_size, env->policy_num_layers);
    env->max_steps = scale_step_limit_for_dt(env->max_steps, reference_dt, dt);
    env->lost_line_limit = scale_step_limit_for_dt(env->lost_line_limit, reference_dt, dt);
    env->dt = dt;
    env->dt_min = fmaxf(0.001f, dt * min_ratio);
    env->dt_max = fmaxf(env->dt_min, dt * max_ratio);
    env->episode_dt = env->dt;
}

static inline float bounded_policy_action(float raw_action) {
    return clampf(raw_action, -1.0f, 1.0f);
}

static inline float raw_action_from_command(float command) {
    return clampf(command, 0.0f, 1.0f);
}

static inline float wheel_ticks_per_sec_to_mps(float ticks_per_sec, float tire_diameter_m) {
    if (ticks_per_sec <= 0.0f || tire_diameter_m <= 0.0f) {
        return 0.0f;
    }
    return ticks_per_sec * (LINE_FOLLOW_PI * tire_diameter_m) / LINE_FOLLOW_TICKS_PER_REV;
}

LineFollowWheelCommand map_actions(float* actions, float max_wheel_speed_mps,
        float command_deadband, float tire_diameter_m,
        float min_drive_ticks_per_sec, float left_speed_scale,
        float right_speed_scale) {
    LineFollowWheelCommand cmd;
    actions[0] = bounded_policy_action(actions[0]);
    actions[1] = bounded_policy_action(actions[1]);

    cmd.left_action = actions[0];
    cmd.right_action = actions[1];
    float left_unit = clampf(cmd.left_action, 0.0f, 1.0f);
    float right_unit = clampf(cmd.right_action, 0.0f, 1.0f);
    if (left_unit < command_deadband) {
        left_unit = 0.0f;
    }
    if (right_unit < command_deadband) {
        right_unit = 0.0f;
    }
    float left_max_mps = fmaxf(0.0f, max_wheel_speed_mps * left_speed_scale);
    float right_max_mps = fmaxf(0.0f, max_wheel_speed_mps * right_speed_scale);
    float min_drive_mps = wheel_ticks_per_sec_to_mps(
        min_drive_ticks_per_sec, tire_diameter_m);
    cmd.left_mps = left_unit * left_max_mps;
    cmd.right_mps = right_unit * right_max_mps;
    if (min_drive_mps > 0.0f) {
        if (left_max_mps > 0.0f) {
            cmd.left_mps = fmaxf(cmd.left_mps, fminf(min_drive_mps, left_max_mps));
        }
        if (right_max_mps > 0.0f) {
            cmd.right_mps = fmaxf(cmd.right_mps, fminf(min_drive_mps, right_max_mps));
        }
    }
    return cmd;
}

static inline void randomize_episode_params(LineFollow* env) {
    env->episode_dt = sample_episode_dt(env);
    env->episode_line_width_m = jittered_positive(
        env->line_width_m, env->line_width_jitter_m, 0.004f, &env->rng);
    env->episode_line_edge_softness_m = jittered_positive(
        env->line_edge_softness_m, env->line_edge_softness_jitter_m, 0.001f, &env->rng);
    env->episode_line_edge_softness_m = fminf(
        env->episode_line_edge_softness_m, 0.75f * env->episode_line_width_m);
}

static inline void jitter_episode_track_line_widths(LineFollow* env) {
    if (env->line_width_segment_jitter_m <= 0.0f) {
        return;
    }

    float offset = env->line_width_segment_jitter_m * rand_signed(&env->rng);
    for (int i = 0; i < env->track.sample_count; i++) {
        float target = env->line_width_segment_jitter_m * rand_signed(&env->rng);
        offset = 0.65f * offset + 0.35f * target;
        env->track.samples[i].line_width_m = fmaxf(
            0.004f, env->episode_line_width_m + offset);
    }
}

static inline void sensor_layout(LineFollow* env) {
    const float lateral[LINE_FOLLOW_SENSOR_COUNT] = {
        env->sensor_side_lateral_m,
        0.0f,
        -env->sensor_side_lateral_m,
    };

    for (int i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++) {
        env->sensor_forward[i] = env->sensor_forward_m
            + env->sensor_forward_jitter_m * rand_signed(&env->rng);
        env->sensor_lateral[i] = lateral[i]
            + env->sensor_lateral_jitter_m * rand_signed(&env->rng);
        env->sensor_gain[i] = 1.0f;
        env->sensor_bias[i] = 0.0f;
        env->sensor_white_time[i] = jittered_positive(
            env->qti_white_time, env->qti_white_jitter, 0.0f, &env->rng);
        env->sensor_black_time[i] = jittered_positive(
            env->qti_black_time, env->qti_black_jitter,
            env->sensor_white_time[i] + 1.0f, &env->rng);
        env->sensor_black_time[i] = fminf(env->sensor_black_time[i], env->qti_timeout);
        env->sensor_black_time[i] = fmaxf(env->sensor_black_time[i],
            env->sensor_white_time[i] + 1.0f);
    }
}

static inline float sample_start_lateral_offset(LineFollow* env) {
    const int active_sensors[LINE_FOLLOW_OBS_SIZE] = {
        LINE_FOLLOW_LEFT,
        LINE_FOLLOW_MIDDLE,
        LINE_FOLLOW_RIGHT,
    };
    const int targets = LINE_FOLLOW_OBS_SIZE;
    int idx = (int)(rand_r(&env->rng) % targets);
    float lateral_offset = 0.0f;
    if (idx < LINE_FOLLOW_OBS_SIZE) {
        lateral_offset = -env->sensor_lateral[active_sensors[idx]];
    }

    float jitter = 0.25f * env->episode_line_width_m * rand_signed(&env->rng);
    float limit = fmaxf(env->start_lateral_offset_m, 0.0f);
    return clampf(lateral_offset + jitter, -limit, limit);
}

void reset_runtime_pose(LineFollow* env, float x, float y, float theta) {
    env->robot_x = x;
    env->robot_y = y;
    env->robot_theta = theta;
    env->v_left = 0.0f;
    env->v_right = 0.0f;
    env->v_left_cmd = 0.0f;
    env->v_right_cmd = 0.0f;
    env->prev_action[0] = 0.0f;
    env->prev_action[1] = 0.0f;
    env->trajectory_count = 0;
}

static inline void record_trajectory(LineFollow* env) {
    if (env->trajectory_count < LINE_FOLLOW_TRAJECTORY_CAP) {
        int idx = env->trajectory_count++;
        env->trajectory_x[idx] = env->robot_x;
        env->trajectory_y[idx] = env->robot_y;
        return;
    }

    memmove(env->trajectory_x, env->trajectory_x + 1,
        (LINE_FOLLOW_TRAJECTORY_CAP - 1) * sizeof(float));
    memmove(env->trajectory_y, env->trajectory_y + 1,
        (LINE_FOLLOW_TRAJECTORY_CAP - 1) * sizeof(float));
    env->trajectory_x[LINE_FOLLOW_TRAJECTORY_CAP - 1] = env->robot_x;
    env->trajectory_y[LINE_FOLLOW_TRAJECTORY_CAP - 1] = env->robot_y;
}

float record_progress_metrics(LineFollow* env, float progress_delta,
        float centerline_error) {
    if (progress_delta <= 0.0f) {
        return 0.0f;
    }

    float accuracy = accuracy_score(env, centerline_error);
    env->effective_progress_m += progress_delta * accuracy;

    if (env->centerline_reward_interval_m <= 1e-6f) {
        return 0.0f;
    }
    env->centerline_reward_progress_m += progress_delta;
    int intervals = (int)(env->centerline_reward_progress_m / env->centerline_reward_interval_m);
    if (intervals <= 0) {
        return 0.0f;
    }

    env->centerline_reward_progress_m -= (float)intervals * env->centerline_reward_interval_m;
    env->perf_quality_sum += (float)intervals * accuracy;
    env->perf_checkpoint_count += intervals;
    return 0.0f;
}

float compute_reward(LineFollow* env, float progress_delta, float centerline_error,
        float heading_error, bool line_seen, float forward_speed, float action_delta,
        float centerline_milestone_reward, float signed_turn, bool idle,
        float negative_action_amount, float action_bound_violation) {
    (void)action_delta;
    (void)centerline_milestone_reward;
    float reward = 0.0f;
    if (progress_delta != 0.0f) {
        float progress_score = progress_fraction_score(env, progress_delta);
        float centerline_score = reward_centerline_score(env, centerline_error);
        reward += env->progress_reward_scale * progress_score * centerline_score;
    }
    if (env->steering_correction_scale > 0.0f
            && fabsf(centerline_error) > 0.001f) {
        float correction_need = clampf(
            fabsf(centerline_error) / fmaxf(env->sensor_side_lateral_m, 1e-6f),
            0.0f, 1.0f);
        float desired_turn = centerline_error < 0.0f ? 1.0f : -1.0f;
        float alignment = clampf(desired_turn * signed_turn, -1.0f, 1.0f);
        reward += env->steering_correction_scale * correction_need * alignment;
    }
    reward -= env->time_penalty;
    reward -= fabsf(centerline_error) * env->centerline_penalty_scale;
    reward -= fabsf(heading_error) * env->heading_penalty_scale;
    if (!line_seen) {
        reward -= env->lost_line_penalty;
    }
    if (idle) {
        reward -= env->idle_penalty;
    }
    if (forward_speed < 0.0f) {
        reward += forward_speed * env->reverse_penalty_scale;
    }
    reward -= negative_action_amount * env->reverse_penalty_scale;
    reward -= action_bound_violation * env->action_bound_penalty_scale;
    return reward;
}

void compute_observations(LineFollow* env) {
    float cos_theta = cosf(env->robot_theta);
    float sin_theta = sinf(env->robot_theta);

    for (int i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++) {
        float forward = env->sensor_forward[i];
        float lateral = env->sensor_lateral[i];
        float sx = env->robot_x + forward * cos_theta - lateral * sin_theta;
        float sy = env->robot_y + forward * sin_theta + lateral * cos_theta;
        env->sensor_x[i] = sx;
        env->sensor_y[i] = sy;

        LineFollowNearest nearest = nearest_track(&env->track, sx, sy);
        float coverage = line_coverage(
            nearest.signed_lateral_m, nearest.line_width_m, env->episode_line_edge_softness_m);
        float raw = env->sensor_white_time[i]
            + coverage * (env->sensor_black_time[i] - env->sensor_white_time[i]);
        raw = raw * env->sensor_gain[i] + env->sensor_bias[i];
        if (env->sensor_noise_std > 0.0f) {
            raw += env->sensor_noise_std * rand_signed(&env->rng);
        }
        raw = clampf(raw, 0.0f, env->qti_timeout);
        env->sensor_raw[i] = raw;
        env->sensor_obs[i] = qti_normalize(raw, env->qti_white_time, env->qti_black_time);
        env->sensor_bits[i] = env->sensor_obs[i] >= env->qti_threshold ? 1 : 0;
    }

    env->observations[0] = env->sensor_obs[LINE_FOLLOW_LEFT];
    env->observations[1] = env->sensor_obs[LINE_FOLLOW_MIDDLE];
    env->observations[2] = env->sensor_obs[LINE_FOLLOW_RIGHT];
}

bool line_visible_to_sensor_array(LineFollow* env) {
    const int active_sensors[LINE_FOLLOW_OBS_SIZE] = {
        LINE_FOLLOW_LEFT,
        LINE_FOLLOW_MIDDLE,
        LINE_FOLLOW_RIGHT,
    };
    float min_lateral = 1e9f;
    float max_lateral = -1e9f;
    float max_edge = 0.0f;
    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        int sensor_idx = active_sensors[i];
        LineFollowNearest nearest = nearest_track(
            &env->track, env->sensor_x[sensor_idx], env->sensor_y[sensor_idx]);
        if (nearest.signed_lateral_m < min_lateral) {
            min_lateral = nearest.signed_lateral_m;
        }
        if (nearest.signed_lateral_m > max_lateral) {
            max_lateral = nearest.signed_lateral_m;
        }
        float edge = 0.5f * nearest.line_width_m + env->episode_line_edge_softness_m;
        if (edge > max_edge) {
            max_edge = edge;
        }
    }
    return min_lateral <= max_edge && max_lateral >= -max_edge;
}

static inline bool generate_episode_track(LineFollow* env) {
    for (int i = 0; i < env->max_track_gen_attempts; i++) {
        if (generate_track(&env->track, &env->rng, env->track_family,
                env->episode_line_width_m, env->track_bounds_m)) {
            jitter_episode_track_line_widths(env);
            return true;
        }
    }
    if (env->track_family == LINE_FOLLOW_TRACK_RANDOM) {
        bool ok = generate_s_curve(&env->track, 0.36f, 0.04f,
            env->episode_line_width_m, env->track_bounds_m);
        if (ok) {
            jitter_episode_track_line_widths(env);
        }
        return ok;
    }
    bool ok = generate_straight(&env->track, env->track_bounds_m * 1.2f,
        env->episode_line_width_m, env->track_bounds_m);
    if (ok) {
        jitter_episode_track_line_widths(env);
    }
    return ok;
}

void c_reset(LineFollow* env) {
    env->tick = 0;
    env->lost_line_steps = 0;
    env->idle_steps = 0;
    env->negative_action_steps = 0;
    env->episode_return = 0.0f;
    env->episode_progress = 0.0f;
    env->effective_progress_m = 0.0f;
    env->center_path_m = 0.0f;
    env->centerline_reward_progress_m = 0.0f;
    env->perf_quality_sum = 0.0f;
    env->perf_checkpoint_count = 0;
    env->forward_speed_sum = 0.0f;
    env->speed_frac_sum = 0.0f;
    env->turn_outer_speed_frac_sum = 0.0f;
    env->action_bound_violation_sum = 0.0f;
    env->raw_action_abs_sum = 0.0f;
    env->last_reward = 0.0f;
    env->centerline_error = 0.0f;
    env->centerline_error_sum = 0.0f;
    env->heading_error = 0.0f;
    env->turn_step_count = 0;
    clear_terminal_info(env);
    env->v_left = 0.0f;
    env->v_right = 0.0f;
    env->v_left_cmd = 0.0f;
    env->v_right_cmd = 0.0f;
    env->prev_action[0] = 0.0f;
    env->prev_action[1] = 0.0f;
    env->trajectory_count = 0;

    randomize_episode_params(env);
    sensor_layout(env);
    generate_episode_track(env);

    const LineFollowTrackSample* start = &env->track.samples[0];
    float start_heading = atan2f(start->tangent_y, start->tangent_x);
    LineFollowNearest nearest;
    bool valid_start = false;

    for (int attempt = 0; attempt < 16; attempt++) {
        float lateral_offset = sample_start_lateral_offset(env);
        float heading_offset = env->start_heading_offset_rad * rand_signed(&env->rng);
        reset_runtime_pose(env,
            start->x + lateral_offset * start->normal_x,
            start->y + lateral_offset * start->normal_y,
            start_heading + heading_offset);
        nearest = nearest_track(&env->track, env->robot_x, env->robot_y);
        env->centerline_error = nearest.signed_lateral_m;
        env->heading_error = angle_diff(env->robot_theta, nearest.heading_rad);
        compute_observations(env);
        if (line_visible_to_sensor_array(env)) {
            valid_start = true;
            break;
        }
    }

    if (!valid_start) {
        reset_runtime_pose(env, start->x, start->y, start_heading);
        nearest = nearest_track(&env->track, env->robot_x, env->robot_y);
        env->centerline_error = nearest.signed_lateral_m;
        env->heading_error = angle_diff(env->robot_theta, nearest.heading_rad);
        compute_observations(env);
    }

    env->last_progress = 0.0f;
    env->episode_progress = 0.0f;
    record_trajectory(env);
}

void c_step(LineFollow* env) {
    float raw_left_action = env->actions[0];
    float raw_right_action = env->actions[1];
    float action_bound_violation = fmaxf(fabsf(raw_left_action) - 1.0f, 0.0f)
        + fmaxf(fabsf(raw_right_action) - 1.0f, 0.0f);
    env->action_bound_violation_sum += action_bound_violation;
    env->raw_action_abs_sum += 0.5f * (fabsf(raw_left_action) + fabsf(raw_right_action));
    LineFollowWheelCommand cmd = map_actions(env->actions,
        env->max_wheel_speed_mps, env->command_deadband,
        env->tire_diameter_m, env->min_drive_ticks_per_sec,
        env->left_speed_scale, env->right_speed_scale);
    env->v_left_cmd = cmd.left_mps;
    env->v_right_cmd = cmd.right_mps;

    float negative_action_amount = fmaxf(-cmd.left_action, 0.0f)
        + fmaxf(-cmd.right_action, 0.0f);
    if (negative_action_amount > 0.0f) {
        env->negative_action_steps += 1;
    }

    float alpha = clampf(env->motor_lag_alpha, 0.0f, 1.0f);
    env->v_left += alpha * (env->v_left_cmd - env->v_left);
    env->v_right += alpha * (env->v_right_cmd - env->v_right);

    float forward_speed = 0.5f * (env->v_left + env->v_right);
    float omega = (env->v_right - env->v_left) / fmaxf(env->wheel_base_m, 1e-4f);
    float dt = env->episode_dt > 0.0f ? env->episode_dt : env->dt;
    env->forward_speed_sum += forward_speed;
    env->speed_frac_sum += clampf(forward_speed / fmaxf(env->max_wheel_speed_mps, 1e-6f),
        0.0f, 1.0f);
    float prev_x = env->robot_x;
    float prev_y = env->robot_y;
    env->robot_x += forward_speed * cosf(env->robot_theta) * dt;
    env->robot_y += forward_speed * sinf(env->robot_theta) * dt;
    env->robot_theta += omega * dt;
    env->robot_theta = angle_diff(env->robot_theta, 0.0f);
    float dx = env->robot_x - prev_x;
    float dy = env->robot_y - prev_y;
    env->center_path_m += sqrtf(dx * dx + dy * dy);

    env->tick += 1;
    compute_observations(env);
    record_trajectory(env);

    LineFollowNearest nearest = nearest_track(&env->track, env->robot_x, env->robot_y);
    float progress_delta = nearest.progress_s - env->last_progress;
    float discontinuity = 0.25f * env->track.length_m;
    if (progress_delta < -discontinuity || progress_delta > discontinuity) {
        progress_delta = 0.0f;
    }
    float max_step_progress = fmaxf(0.002f, env->max_wheel_speed_mps * dt * 3.0f);
    progress_delta = clampf(progress_delta, -max_step_progress, max_step_progress);

    env->last_progress = nearest.progress_s;
    if (progress_delta > 0.0f) {
        env->episode_progress = fminf(env->track.length_m,
            env->episode_progress + progress_delta);
    }
    env->centerline_error = nearest.signed_lateral_m;
    env->heading_error = angle_diff(env->robot_theta, nearest.heading_rad);
    env->centerline_error_sum += fabsf(env->centerline_error);

    bool line_seen = line_visible_to_sensor_array(env);
    env->lost_line_steps = line_seen ? 0 : env->lost_line_steps + 1;

    float action_delta = fabsf(cmd.left_action - env->prev_action[0])
        + fabsf(cmd.right_action - env->prev_action[1]);
    env->prev_action[0] = cmd.left_action;
    env->prev_action[1] = cmd.right_action;
    float signed_turn = clampf(
        (cmd.right_mps - cmd.left_mps) / fmaxf(env->max_wheel_speed_mps, 1e-6f),
        -1.0f, 1.0f);
    if (fabsf(signed_turn) >= 0.25f) {
        float outer_mps = signed_turn > 0.0f ? cmd.right_mps : cmd.left_mps;
        env->turn_outer_speed_frac_sum += clampf(
            outer_mps / fmaxf(env->max_wheel_speed_mps, 1e-6f), 0.0f, 1.0f);
        env->turn_step_count += 1;
    }
    float command_avg_mps = 0.5f * (cmd.left_mps + cmd.right_mps);
    bool idle = command_avg_mps
        < env->min_wheel_action * fmaxf(env->max_wheel_speed_mps, 1e-6f);
    if (idle) {
        env->idle_steps += 1;
    }
    float centerline_milestone_reward = record_progress_metrics(
        env, progress_delta, env->centerline_error);

    float reward = compute_reward(env, progress_delta, env->centerline_error,
        env->heading_error, line_seen, forward_speed, action_delta,
        centerline_milestone_reward, signed_turn, idle, negative_action_amount,
        action_bound_violation);
    if (env->turn_speed_penalty_scale > 0.0f
            && fabsf(env->centerline_error) > 0.001f) {
        float correction_need = clampf(
            fabsf(env->centerline_error) / fmaxf(env->sensor_side_lateral_m, 1e-6f),
            0.0f, 1.0f);
        float desired_turn = env->centerline_error < 0.0f ? 1.0f : -1.0f;
        float desired_outer_mps = desired_turn > 0.0f ? cmd.right_mps : cmd.left_mps;
        float desired_outer_frac = clampf(
            desired_outer_mps / fmaxf(env->max_wheel_speed_mps, 1e-6f), 0.0f, 1.0f);
        float shortfall = fmaxf(env->min_turn_outer_action - desired_outer_frac, 0.0f);
        reward -= env->turn_speed_penalty_scale * correction_need * shortfall;
    }
    reward = clampf(reward, -1.0f, 1.0f);
    env->rewards[0] = reward;
    env->last_reward = reward;
    env->episode_return += reward;

    bool timeout = env->tick >= env->max_steps;
    bool lost_line = env->lost_line_steps >= env->lost_line_limit;
    bool too_far = fabsf(env->centerline_error) > env->too_far_m;
    bool track_complete = env->episode_progress + env->track_complete_margin_m
        >= env->track.length_m;
    bool done = timeout || lost_line || too_far || track_complete;
    float success_quality = episode_perf_score(env);
    bool success = (timeout || track_complete) && !lost_line && !too_far
        && env->idle_steps == 0 && success_quality > 0.0f;

    env->terminals[0] = done ? 1.0f : 0.0f;
    if (done) {
        clear_terminal_info(env);
        env->terminal_timeout = timeout ? 1.0f : 0.0f;
        env->terminal_lost_line = lost_line ? 1.0f : 0.0f;
        env->terminal_too_far = too_far ? 1.0f : 0.0f;
        env->terminal_track_complete = track_complete ? 1.0f : 0.0f;
        if (lost_line || too_far) {
            float previous_reward = env->rewards[0];
            env->rewards[0] = clampf(-fabsf(env->off_track_terminal_penalty), -1.0f, 0.0f);
            env->episode_return += env->rewards[0] - previous_reward;
            env->last_reward = env->rewards[0];
        } else if (success) {
            float terminal_bonus = env->success_reward * success_quality;
            float final_reward = clampf(env->rewards[0] + terminal_bonus, -1.0f, 1.0f);
            env->episode_return += final_reward - env->rewards[0];
            env->rewards[0] = final_reward;
            env->last_reward = final_reward;
            env->terminal_success = success_quality;
        }
        add_log(env);
        c_reset(env);
    }
}

#ifndef LINE_FOLLOW_NO_RENDER
static inline Vector2 world_to_screen(LineFollow* env, float x, float y,
        int width, int height, float scale) {
    return (Vector2){
        width * 0.5f + (x - env->robot_x) * scale,
        height * 0.58f - (y - env->robot_y) * scale,
    };
}

static inline Vector2 body_to_screen(LineFollow* env, float forward_m, float lateral_m,
        int width, int height, float scale) {
    float cos_theta = cosf(env->robot_theta);
    float sin_theta = sinf(env->robot_theta);
    float x = env->robot_x + forward_m * cos_theta - lateral_m * sin_theta;
    float y = env->robot_y + forward_m * sin_theta + lateral_m * cos_theta;
    return world_to_screen(env, x, y, width, height, scale);
}

static inline void draw_body_rect(LineFollow* env, float forward_m, float lateral_m,
        float length_m, float width_m, int screen_width, int screen_height, float scale, Color color) {
    Vector2 center = body_to_screen(env, forward_m, lateral_m,
        screen_width, screen_height, scale);
    float length_px = fmaxf(1.0f, length_m * scale);
    float width_px = fmaxf(1.0f, width_m * scale);
    Rectangle rect = {
        center.x,
        center.y,
        length_px,
        width_px,
    };
    DrawRectanglePro(rect, (Vector2){length_px * 0.5f, width_px * 0.5f},
        -env->robot_theta * 180.0f / LINE_FOLLOW_PI, color);
}

void c_render(LineFollow* env) {
    if (!IsWindowReady()) {
        InitWindow(1000, 760, "PufferLib Line Follow");
        SetTargetFPS(60);
    }
    if (IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }
    if (IsKeyPressed(KEY_TAB)) {
        ToggleFullscreen();
    }

    int width = GetScreenWidth();
    int height = GetScreenHeight();
    float scale = fminf(width, height) / 0.65f;

    BeginDrawing();
    ClearBackground((Color){248, 248, 244, 255});

    for (int i = 0; i < env->track.sample_count - 1; i++) {
        Vector2 a = world_to_screen(env,
            env->track.samples[i].x, env->track.samples[i].y, width, height, scale);
        Vector2 b = world_to_screen(env,
            env->track.samples[i + 1].x, env->track.samples[i + 1].y, width, height, scale);
        float line_width = 0.5f * (
            env->track.samples[i].line_width_m
            + env->track.samples[i + 1].line_width_m);
        DrawLineEx(a, b, fmaxf(2.0f, line_width * scale), BLACK);
    }
    for (int i = 0; i < env->track.sample_count; i++) {
        Vector2 p = world_to_screen(env,
            env->track.samples[i].x, env->track.samples[i].y, width, height, scale);
        DrawCircleV(p, fmaxf(1.0f, 0.5f * env->track.samples[i].line_width_m * scale), BLACK);
    }

    for (int i = 1; i < env->trajectory_count; i++) {
        Vector2 a = world_to_screen(env,
            env->trajectory_x[i - 1], env->trajectory_y[i - 1], width, height, scale);
        Vector2 b = world_to_screen(env,
            env->trajectory_x[i], env->trajectory_y[i], width, height, scale);
        DrawLineEx(a, b, 2.0f, (Color){40, 140, 200, 120});
    }

    Vector2 center = world_to_screen(env, env->robot_x, env->robot_y, width, height, scale);
    float tire_lateral = env->wheel_base_m * 0.5f;
    draw_body_rect(env, 0.0f, tire_lateral, env->tire_diameter_m,
        env->tire_width_m, width, height, scale, (Color){18, 18, 18, 255});
    draw_body_rect(env, 0.0f, -tire_lateral, env->tire_diameter_m,
        env->tire_width_m, width, height, scale, (Color){18, 18, 18, 255});

    float body_len_m = env->body_ahead_m + env->body_behind_m;
    float body_center_forward_m = 0.5f * (env->body_ahead_m - env->body_behind_m);
    draw_body_rect(env, body_center_forward_m, 0.0f, body_len_m,
        env->body_width_m, width, height, scale, (Color){42, 78, 96, 255});

    Vector2 axle_left = body_to_screen(env, 0.0f, tire_lateral, width, height, scale);
    Vector2 axle_right = body_to_screen(env, 0.0f, -tire_lateral, width, height, scale);
    DrawLineEx(axle_left, axle_right, 2.0f, (Color){210, 218, 220, 180});
    DrawCircleV(center, 3.5f, (Color){245, 245, 240, 255});

    Vector2 nose = world_to_screen(env,
        env->robot_x + env->body_ahead_m * cosf(env->robot_theta),
        env->robot_y + env->body_ahead_m * sinf(env->robot_theta), width, height, scale);
    DrawLineEx(center, nose, 3.0f, (Color){230, 90, 55, 255});

    for (int i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++) {
        Vector2 p = world_to_screen(env, env->sensor_x[i], env->sensor_y[i],
            width, height, scale);
        unsigned char shade = (unsigned char)(255.0f * (1.0f - env->sensor_obs[i]));
        Color color = (Color){shade, shade, shade, 255};
        DrawCircleV(p, 7.0f, color);
        DrawCircleLines((int)p.x, (int)p.y, 7.0f, env->sensor_bits[i] ? RED : GRAY);
    }

    DrawText(TextFormat("step %d  dt %.0fms  reward %.3f  return %.2f", env->tick,
        1000.0f * env->episode_dt, env->last_reward, env->episode_return),
        16, 16, 20, (Color){20, 20, 20, 255});
    DrawText(TextFormat("progress %.2fm / %.2fm  error %.3fm  heading %.2frad",
        env->episode_progress, env->track.length_m, env->centerline_error, env->heading_error),
        16, 42, 20, (Color){20, 20, 20, 255});
    DrawText(TextFormat("action L %.2f R %.2f  lost %d  qti L %.2f M %.2f R %.2f",
        env->actions[0], env->actions[1], env->lost_line_steps,
        env->observations[0], env->observations[1], env->observations[2]),
        16, 68, 20, (Color){20, 20, 20, 255});

    EndDrawing();
}
#else
void c_render(LineFollow* env) {
    (void)env;
}
#endif

void c_close(LineFollow* env) {
    (void)env;
#ifndef LINE_FOLLOW_NO_RENDER
    if (IsWindowReady()) {
        CloseWindow();
    }
#endif
}

#endif
