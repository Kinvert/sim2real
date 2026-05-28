#include "line_follow.h"

#define OBS_SIZE 4
#define NUM_ATNS 2
#define ACT_SIZES {1, 1}
#define OBS_TENSOR_T FloatTensor

#define Env LineFollow
#include "vecenv.h"

void my_init(Env* env, Dict* kwargs) {
    set_defaults(env);
    env->num_agents = 1;

    env->dt = dict_get(kwargs, "dt")->value;
    env->max_steps = dict_get(kwargs, "max_steps")->value;
    env->max_wheel_speed_mps = dict_get(kwargs, "max_wheel_speed_mps")->value;
    env->wheel_base_m = dict_get(kwargs, "wheel_base_m")->value;
    env->body_ahead_m = dict_get(kwargs, "body_ahead_m")->value;
    env->body_behind_m = dict_get(kwargs, "body_behind_m")->value;
    env->body_width_m = dict_get(kwargs, "body_width_m")->value;
    env->tire_diameter_m = dict_get(kwargs, "tire_diameter_m")->value;
    env->tire_width_m = dict_get(kwargs, "tire_width_m")->value;
    env->motor_lag_alpha = dict_get(kwargs, "motor_lag_alpha")->value;
    env->command_deadband = dict_get(kwargs, "command_deadband")->value;
    env->left_speed_scale = dict_get(kwargs, "left_speed_scale")->value;
    env->right_speed_scale = dict_get(kwargs, "right_speed_scale")->value;
    env->lost_line_limit = dict_get(kwargs, "lost_line_limit")->value;
    env->track_family = dict_get(kwargs, "track_family")->value;
    env->max_track_gen_attempts = dict_get(kwargs, "max_track_gen_attempts")->value;

    env->progress_reward_scale = dict_get(kwargs, "progress_reward_scale")->value;
    env->centerline_penalty_scale = dict_get(kwargs, "centerline_penalty_scale")->value;
    env->heading_penalty_scale = dict_get(kwargs, "heading_penalty_scale")->value;
    env->lost_line_penalty = dict_get(kwargs, "lost_line_penalty")->value;
    env->action_smoothness_penalty = dict_get(kwargs, "action_smoothness_penalty")->value;
    env->reverse_penalty_scale = dict_get(kwargs, "reverse_penalty_scale")->value;
    env->too_far_m = dict_get(kwargs, "too_far_m")->value;
    env->track_complete_margin_m = dict_get(kwargs, "track_complete_margin_m")->value;

    env->sensor_forward_m = dict_get(kwargs, "sensor_forward_m")->value;
    env->sensor_inner_lateral_m = dict_get(kwargs, "sensor_inner_lateral_m")->value;
    env->sensor_outer_lateral_m = dict_get(kwargs, "sensor_outer_lateral_m")->value;
    env->sensor_lateral_jitter_m = dict_get(kwargs, "sensor_lateral_jitter_m")->value;
    env->sensor_forward_jitter_m = dict_get(kwargs, "sensor_forward_jitter_m")->value;
    env->sensor_noise_std = dict_get(kwargs, "sensor_noise_std")->value;
    env->qti_threshold = dict_get(kwargs, "qti_threshold")->value;
    env->qti_white_time = dict_get(kwargs, "qti_white_time")->value;
    env->qti_black_time = dict_get(kwargs, "qti_black_time")->value;
    env->qti_timeout = dict_get(kwargs, "qti_timeout")->value;

    env->line_width_m = dict_get(kwargs, "line_width_m")->value;
    env->line_edge_softness_m = dict_get(kwargs, "line_edge_softness_m")->value;
    env->track_bounds_m = dict_get(kwargs, "track_bounds_m")->value;
    env->start_lateral_offset_m = dict_get(kwargs, "start_lateral_offset_m")->value;
    env->start_heading_offset_rad = dict_get(kwargs, "start_heading_offset_rad")->value;

    init(env);
}

void my_log(Log* log, Dict* out) {
    dict_set(out, "perf", log->perf);
    dict_set(out, "score", log->score);
    dict_set(out, "episode_return", log->episode_return);
    dict_set(out, "episode_length", log->episode_length);
    dict_set(out, "centerline_error", log->centerline_error);
    dict_set(out, "n", log->n);
}
