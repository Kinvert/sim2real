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

#define LINE_FOLLOW_OBS_SIZE 4
#define LINE_FOLLOW_NUM_ATNS 2
#define LINE_FOLLOW_TRAJECTORY_CAP 512

typedef struct Log Log;
struct Log {
    float perf;
    float score;
    float episode_return;
    float episode_length;
    float centerline_error;
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
    int lost_line_limit;
    int track_family;
    int max_track_gen_attempts;

    float dt;
    float max_wheel_speed_mps;
    float wheel_base_m;
    float motor_lag_alpha;
    float command_deadband;
    float left_speed_scale;
    float right_speed_scale;

    float robot_x;
    float robot_y;
    float robot_theta;
    float v_left;
    float v_right;
    float v_left_cmd;
    float v_right_cmd;
    float prev_action[2];

    float episode_return;
    float episode_progress;
    float last_progress;
    float centerline_error;
    float centerline_error_sum;
    float heading_error;
    float last_reward;

    float progress_reward_scale;
    float centerline_penalty_scale;
    float heading_penalty_scale;
    float lost_line_penalty;
    float action_smoothness_penalty;
    float reverse_penalty_scale;
    float too_far_m;
    float track_complete_margin_m;

    float sensor_forward_m;
    float sensor_inner_lateral_m;
    float sensor_outer_lateral_m;
    float sensor_lateral_jitter_m;
    float sensor_forward_jitter_m;
    float sensor_noise_std;
    float sensor_gain[LINE_FOLLOW_OBS_SIZE];
    float sensor_bias[LINE_FOLLOW_OBS_SIZE];
    float sensor_forward[LINE_FOLLOW_OBS_SIZE];
    float sensor_lateral[LINE_FOLLOW_OBS_SIZE];
    float sensor_x[LINE_FOLLOW_OBS_SIZE];
    float sensor_y[LINE_FOLLOW_OBS_SIZE];
    float sensor_raw[LINE_FOLLOW_OBS_SIZE];
    unsigned char sensor_bits[LINE_FOLLOW_OBS_SIZE];
    float qti_threshold;

    float qti_white_time;
    float qti_black_time;
    float qti_timeout;
    float line_width_m;
    float line_edge_softness_m;
    float track_bounds_m;
    float start_lateral_offset_m;
    float start_heading_offset_rad;

    float trajectory_x[LINE_FOLLOW_TRAJECTORY_CAP];
    float trajectory_y[LINE_FOLLOW_TRAJECTORY_CAP];
    int trajectory_count;
};

static inline float line_follow_clampf(float value, float lo, float hi) {
    if (!isfinite(value)) return 0.0f;
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static inline float line_follow_rand_signed(unsigned int* rng) {
    return 2.0f * ((float)rand_r(rng) / (float)RAND_MAX) - 1.0f;
}

void line_follow_set_defaults(LineFollow* env) {
    env->num_agents = 1;
    env->max_steps = 600;
    env->lost_line_limit = 30;
    env->track_family = LINE_FOLLOW_TRACK_RANDOM;
    env->max_track_gen_attempts = 4;

    env->dt = 0.05f;
    env->max_wheel_speed_mps = 0.35f;
    env->wheel_base_m = 0.105f;
    env->motor_lag_alpha = 0.65f;
    env->command_deadband = 0.04f;
    env->left_speed_scale = 1.0f;
    env->right_speed_scale = 1.0f;

    env->progress_reward_scale = 8.0f;
    env->centerline_penalty_scale = 4.0f;
    env->heading_penalty_scale = 0.05f;
    env->lost_line_penalty = 0.04f;
    env->action_smoothness_penalty = 0.015f;
    env->reverse_penalty_scale = 0.2f;
    env->too_far_m = 0.09f;
    env->track_complete_margin_m = 0.06f;

    env->sensor_forward_m = 0.055f;
    env->sensor_inner_lateral_m = 0.012f;
    env->sensor_outer_lateral_m = 0.035f;
    env->sensor_lateral_jitter_m = 0.0f;
    env->sensor_forward_jitter_m = 0.0f;
    env->sensor_noise_std = 0.0f;
    env->qti_threshold = 0.5f;
    env->qti_white_time = 100.0f;
    env->qti_black_time = 1100.0f;
    env->qti_timeout = 2000.0f;
    env->line_width_m = 0.018f;
    env->line_edge_softness_m = 0.006f;
    env->track_bounds_m = 1.0f;
    env->start_lateral_offset_m = 0.015f;
    env->start_heading_offset_rad = 0.12f;

    const float lateral[LINE_FOLLOW_OBS_SIZE] = {
        env->sensor_outer_lateral_m,
        env->sensor_inner_lateral_m,
        -env->sensor_inner_lateral_m,
        -env->sensor_outer_lateral_m,
    };
    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        env->sensor_gain[i] = 1.0f;
        env->sensor_bias[i] = 0.0f;
        env->sensor_forward[i] = env->sensor_forward_m;
        env->sensor_lateral[i] = lateral[i];
    }
}

void init(LineFollow* env) {
    memset(&env->log, 0, sizeof(Log));
    env->tick = 0;
    env->lost_line_steps = 0;
    env->episode_return = 0.0f;
    env->episode_progress = 0.0f;
    env->last_progress = 0.0f;
    env->centerline_error = 0.0f;
    env->centerline_error_sum = 0.0f;
    env->heading_error = 0.0f;
    env->last_reward = 0.0f;
    env->v_left = 0.0f;
    env->v_right = 0.0f;
    env->v_left_cmd = 0.0f;
    env->v_right_cmd = 0.0f;
    env->prev_action[0] = 0.0f;
    env->prev_action[1] = 0.0f;
    env->trajectory_count = 0;
}

void allocate(LineFollow* env) {
    line_follow_set_defaults(env);
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

void add_log(LineFollow* env) {
    float episode_length = (float)env->tick;
    float mean_error = episode_length > 0.0f ? env->centerline_error_sum / episode_length : 0.0f;
    float perf = env->track.length_m > 1e-6f
        ? line_follow_clampf(env->episode_progress / env->track.length_m, 0.0f, 1.0f)
        : 0.0f;

    env->log.perf += perf;
    env->log.score += env->episode_progress;
    env->log.episode_return += env->episode_return;
    env->log.episode_length += episode_length;
    env->log.centerline_error += mean_error;
    env->log.n += 1.0f;
}

float line_follow_qti_normalize(float raw, float white_time, float black_time) {
    float denom = black_time - white_time;
    if (fabsf(denom) < 1e-6f) {
        return raw >= black_time ? 1.0f : 0.0f;
    }
    return line_follow_clampf((raw - white_time) / denom, 0.0f, 1.0f);
}

LineFollowWheelCommand line_follow_map_actions(float* actions, float max_wheel_speed_mps,
        float command_deadband, float left_speed_scale, float right_speed_scale) {
    LineFollowWheelCommand cmd;
    actions[0] = line_follow_clampf(actions[0], -1.0f, 1.0f);
    actions[1] = line_follow_clampf(actions[1], -1.0f, 1.0f);

    cmd.left_action = fabsf(actions[0]) < command_deadband ? 0.0f : actions[0];
    cmd.right_action = fabsf(actions[1]) < command_deadband ? 0.0f : actions[1];
    cmd.left_mps = cmd.left_action * max_wheel_speed_mps * left_speed_scale;
    cmd.right_mps = cmd.right_action * max_wheel_speed_mps * right_speed_scale;
    return cmd;
}

static inline void line_follow_sensor_layout(LineFollow* env) {
    const float lateral[LINE_FOLLOW_OBS_SIZE] = {
        env->sensor_outer_lateral_m,
        env->sensor_inner_lateral_m,
        -env->sensor_inner_lateral_m,
        -env->sensor_outer_lateral_m,
    };

    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        env->sensor_forward[i] = env->sensor_forward_m
            + env->sensor_forward_jitter_m * line_follow_rand_signed(&env->rng);
        env->sensor_lateral[i] = lateral[i]
            + env->sensor_lateral_jitter_m * line_follow_rand_signed(&env->rng);
        env->sensor_gain[i] = 1.0f;
        env->sensor_bias[i] = 0.0f;
    }
}

void line_follow_reset_runtime_pose(LineFollow* env, float x, float y, float theta) {
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

static inline void line_follow_record_trajectory(LineFollow* env) {
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

float line_follow_compute_reward(LineFollow* env, float progress_delta, float centerline_error,
        float heading_error, bool line_seen, float forward_speed, float action_delta) {
    float reward = progress_delta * env->progress_reward_scale;
    reward -= fabsf(centerline_error) * env->centerline_penalty_scale;
    reward -= fabsf(heading_error) * env->heading_penalty_scale;
    if (!line_seen) {
        reward -= env->lost_line_penalty;
    }
    reward -= action_delta * env->action_smoothness_penalty;
    if (forward_speed < 0.0f) {
        reward += forward_speed * env->reverse_penalty_scale;
    }
    return reward;
}

void compute_observations(LineFollow* env) {
    float cos_theta = cosf(env->robot_theta);
    float sin_theta = sinf(env->robot_theta);

    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        float forward = env->sensor_forward[i];
        float lateral = env->sensor_lateral[i];
        float sx = env->robot_x + forward * cos_theta - lateral * sin_theta;
        float sy = env->robot_y + forward * sin_theta + lateral * cos_theta;
        env->sensor_x[i] = sx;
        env->sensor_y[i] = sy;

        LineFollowNearest nearest = line_follow_track_nearest(&env->track, sx, sy);
        float coverage = line_follow_track_coverage(
            nearest.signed_lateral_m, nearest.line_width_m, env->line_edge_softness_m);
        float raw = env->qti_white_time + coverage * (env->qti_black_time - env->qti_white_time);
        raw = raw * env->sensor_gain[i] + env->sensor_bias[i];
        if (env->sensor_noise_std > 0.0f) {
            raw += env->sensor_noise_std * line_follow_rand_signed(&env->rng);
        }
        raw = line_follow_clampf(raw, 0.0f, env->qti_timeout);
        env->sensor_raw[i] = raw;
        env->observations[i] = line_follow_qti_normalize(raw, env->qti_white_time, env->qti_black_time);
        env->sensor_bits[i] = env->observations[i] >= env->qti_threshold ? 1 : 0;
    }
}

static inline bool line_follow_generate_episode_track(LineFollow* env) {
    for (int i = 0; i < env->max_track_gen_attempts; i++) {
        if (line_follow_track_generate(&env->track, &env->rng, env->track_family,
                env->line_width_m, env->track_bounds_m)) {
            return true;
        }
    }
    return line_follow_track_generate_straight(&env->track, env->track_bounds_m * 1.2f,
        env->line_width_m, env->track_bounds_m);
}

void c_reset(LineFollow* env) {
    env->tick = 0;
    env->lost_line_steps = 0;
    env->episode_return = 0.0f;
    env->episode_progress = 0.0f;
    env->last_reward = 0.0f;
    env->centerline_error = 0.0f;
    env->centerline_error_sum = 0.0f;
    env->heading_error = 0.0f;
    env->v_left = 0.0f;
    env->v_right = 0.0f;
    env->v_left_cmd = 0.0f;
    env->v_right_cmd = 0.0f;
    env->prev_action[0] = 0.0f;
    env->prev_action[1] = 0.0f;
    env->trajectory_count = 0;

    line_follow_sensor_layout(env);
    line_follow_generate_episode_track(env);

    const LineFollowTrackSample* start = &env->track.samples[0];
    float lateral_offset = env->start_lateral_offset_m * line_follow_rand_signed(&env->rng);
    float heading_offset = env->start_heading_offset_rad * line_follow_rand_signed(&env->rng);
    float start_heading = atan2f(start->tangent_y, start->tangent_x);
    line_follow_reset_runtime_pose(env,
        start->x + lateral_offset * start->normal_x,
        start->y + lateral_offset * start->normal_y,
        start_heading + heading_offset);

    LineFollowNearest nearest = line_follow_track_nearest(&env->track, env->robot_x, env->robot_y);
    env->last_progress = nearest.progress_s;
    env->episode_progress = nearest.progress_s;
    env->centerline_error = nearest.signed_lateral_m;
    env->heading_error = line_follow_angle_diff(env->robot_theta, nearest.heading_rad);
    compute_observations(env);
    line_follow_record_trajectory(env);
}

void c_step(LineFollow* env) {
    LineFollowWheelCommand cmd = line_follow_map_actions(env->actions,
        env->max_wheel_speed_mps, env->command_deadband,
        env->left_speed_scale, env->right_speed_scale);
    env->v_left_cmd = cmd.left_mps;
    env->v_right_cmd = cmd.right_mps;

    float alpha = line_follow_clampf(env->motor_lag_alpha, 0.0f, 1.0f);
    env->v_left += alpha * (env->v_left_cmd - env->v_left);
    env->v_right += alpha * (env->v_right_cmd - env->v_right);

    float forward_speed = 0.5f * (env->v_left + env->v_right);
    float omega = (env->v_right - env->v_left) / fmaxf(env->wheel_base_m, 1e-4f);
    env->robot_x += forward_speed * cosf(env->robot_theta) * env->dt;
    env->robot_y += forward_speed * sinf(env->robot_theta) * env->dt;
    env->robot_theta += omega * env->dt;
    env->robot_theta = line_follow_angle_diff(env->robot_theta, 0.0f);

    env->tick += 1;
    compute_observations(env);
    line_follow_record_trajectory(env);

    LineFollowNearest nearest = line_follow_track_nearest(&env->track, env->robot_x, env->robot_y);
    float progress_delta = nearest.progress_s - env->last_progress;
    if (progress_delta < -0.25f * env->track.length_m) {
        progress_delta = 0.0f;
    }
    progress_delta = line_follow_clampf(progress_delta, -0.05f, 0.08f);

    env->last_progress = nearest.progress_s;
    if (nearest.progress_s > env->episode_progress) {
        env->episode_progress = nearest.progress_s;
    }
    env->centerline_error = nearest.signed_lateral_m;
    env->heading_error = line_follow_angle_diff(env->robot_theta, nearest.heading_rad);
    env->centerline_error_sum += fabsf(env->centerline_error);

    bool line_seen = false;
    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        if (env->observations[i] >= env->qti_threshold) {
            line_seen = true;
        }
    }
    env->lost_line_steps = line_seen ? 0 : env->lost_line_steps + 1;

    float action_delta = fabsf(cmd.left_action - env->prev_action[0])
        + fabsf(cmd.right_action - env->prev_action[1]);
    env->prev_action[0] = cmd.left_action;
    env->prev_action[1] = cmd.right_action;

    float reward = line_follow_compute_reward(env, progress_delta, env->centerline_error,
        env->heading_error, line_seen, forward_speed, action_delta);
    env->rewards[0] = reward;
    env->last_reward = reward;
    env->episode_return += reward;

    bool done = false;
    if (env->tick >= env->max_steps) {
        done = true;
    }
    if (env->lost_line_steps >= env->lost_line_limit) {
        done = true;
    }
    if (fabsf(env->centerline_error) > env->too_far_m) {
        done = true;
    }
    if (env->episode_progress + env->track_complete_margin_m >= env->track.length_m) {
        done = true;
    }

    env->terminals[0] = done ? 1.0f : 0.0f;
    if (done) {
        add_log(env);
        c_reset(env);
    }
}

#ifndef LINE_FOLLOW_NO_RENDER
static inline Vector2 line_follow_world_to_screen(LineFollow* env, float x, float y,
        int width, int height, float scale) {
    (void)env;
    return (Vector2){
        width * 0.5f + x * scale,
        height * 0.58f - y * scale,
    };
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
    float scale = fminf(width, height) / (2.5f * env->track_bounds_m);

    BeginDrawing();
    ClearBackground((Color){248, 248, 244, 255});

    for (int i = 0; i < env->track.sample_count - 1; i++) {
        Vector2 a = line_follow_world_to_screen(env,
            env->track.samples[i].x, env->track.samples[i].y, width, height, scale);
        Vector2 b = line_follow_world_to_screen(env,
            env->track.samples[i + 1].x, env->track.samples[i + 1].y, width, height, scale);
        DrawLineEx(a, b, fmaxf(2.0f, env->track.samples[i].line_width_m * scale), BLACK);
    }

    for (int i = 1; i < env->trajectory_count; i++) {
        Vector2 a = line_follow_world_to_screen(env,
            env->trajectory_x[i - 1], env->trajectory_y[i - 1], width, height, scale);
        Vector2 b = line_follow_world_to_screen(env,
            env->trajectory_x[i], env->trajectory_y[i], width, height, scale);
        DrawLineEx(a, b, 2.0f, (Color){40, 140, 200, 120});
    }

    Vector2 center = line_follow_world_to_screen(env, env->robot_x, env->robot_y, width, height, scale);
    float body_len = 0.095f * scale;
    float body_w = 0.075f * scale;
    Rectangle body = {
        center.x - body_len * 0.5f,
        center.y - body_w * 0.5f,
        body_len,
        body_w,
    };
    DrawRectanglePro(body, (Vector2){body_len * 0.5f, body_w * 0.5f},
        -env->robot_theta * 180.0f / LINE_FOLLOW_PI, (Color){42, 78, 96, 255});

    Vector2 nose = line_follow_world_to_screen(env,
        env->robot_x + 0.07f * cosf(env->robot_theta),
        env->robot_y + 0.07f * sinf(env->robot_theta),
        width, height, scale);
    DrawLineEx(center, nose, 3.0f, (Color){230, 90, 55, 255});

    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        Vector2 p = line_follow_world_to_screen(env, env->sensor_x[i], env->sensor_y[i],
            width, height, scale);
        unsigned char shade = (unsigned char)(255.0f * (1.0f - env->observations[i]));
        Color color = (Color){shade, shade, shade, 255};
        DrawCircleV(p, 7.0f, color);
        DrawCircleLines((int)p.x, (int)p.y, 7.0f, env->sensor_bits[i] ? RED : GRAY);
    }

    DrawText(TextFormat("step %d  reward %.3f  return %.2f", env->tick,
        env->last_reward, env->episode_return), 16, 16, 20, (Color){20, 20, 20, 255});
    DrawText(TextFormat("progress %.2fm / %.2fm  error %.3fm  heading %.2frad",
        env->episode_progress, env->track.length_m, env->centerline_error, env->heading_error),
        16, 42, 20, (Color){20, 20, 20, 255});
    DrawText(TextFormat("action L %.2f R %.2f  lost %d  qti %.2f %.2f %.2f %.2f",
        env->actions[0], env->actions[1], env->lost_line_steps,
        env->observations[0], env->observations[1], env->observations[2], env->observations[3]),
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
