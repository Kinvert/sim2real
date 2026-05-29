#include "line_follow.h"

#define OBS_SIZE 3
#define NUM_ATNS 2
#define ACT_SIZES {1, 1}
#define OBS_TENSOR_T FloatTensor

#define Env LineFollow
#include "vecenv.h"

void my_init(Env* env, Dict* kwargs) {
    set_defaults(env);
    env->num_agents = 1;

    env->dt = dict_get(kwargs, "dt")->value;
    env->dt_min = dict_get(kwargs, "dt_min")->value;
    env->dt_max = dict_get(kwargs, "dt_max")->value;
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
    env->policy_hidden_size = dict_get(kwargs, "policy_hidden_size")->value;
    env->policy_num_layers = dict_get(kwargs, "policy_num_layers")->value;
    env->model_dt_enabled = dict_get(kwargs, "model_dt_enabled")->value;

    env->progress_reward_scale = dict_get(kwargs, "progress_reward_scale")->value;
    env->centerline_penalty_scale = dict_get(kwargs, "centerline_penalty_scale")->value;
    env->centerline_reward_interval_m = dict_get(kwargs, "centerline_reward_interval_m")->value;
    env->success_reward = dict_get(kwargs, "success_reward")->value;
    env->heading_penalty_scale = dict_get(kwargs, "heading_penalty_scale")->value;
    env->lost_line_penalty = dict_get(kwargs, "lost_line_penalty")->value;
    env->off_track_terminal_penalty = dict_get(kwargs, "off_track_terminal_penalty")->value;
    env->action_smoothness_penalty = dict_get(kwargs, "action_smoothness_penalty")->value;
    env->action_bound_penalty_scale = dict_get(kwargs, "action_bound_penalty_scale")->value;
    env->turn_penalty_scale = dict_get(kwargs, "turn_penalty_scale")->value;
    env->steering_correction_scale = dict_get(kwargs, "steering_correction_scale")->value;
    env->turn_speed_penalty_scale = dict_get(kwargs, "turn_speed_penalty_scale")->value;
    env->min_turn_outer_action = dict_get(kwargs, "min_turn_outer_action")->value;
    env->time_penalty = dict_get(kwargs, "time_penalty")->value;
    env->idle_penalty = dict_get(kwargs, "idle_penalty")->value;
    env->min_wheel_action = dict_get(kwargs, "min_wheel_action")->value;
    env->reverse_penalty_scale = dict_get(kwargs, "reverse_penalty_scale")->value;
    env->too_far_m = dict_get(kwargs, "too_far_m")->value;
    env->track_complete_margin_m = dict_get(kwargs, "track_complete_margin_m")->value;

    env->sensor_forward_m = dict_get(kwargs, "sensor_forward_m")->value;
    env->sensor_side_lateral_m = dict_get(kwargs, "sensor_side_lateral_m")->value;
    env->sensor_lateral_jitter_m = dict_get(kwargs, "sensor_lateral_jitter_m")->value;
    env->sensor_forward_jitter_m = dict_get(kwargs, "sensor_forward_jitter_m")->value;
    env->sensor_noise_std = dict_get(kwargs, "sensor_noise_std")->value;
    env->qti_threshold = dict_get(kwargs, "qti_threshold")->value;
    env->qti_white_time = dict_get(kwargs, "qti_white_time")->value;
    env->qti_black_time = dict_get(kwargs, "qti_black_time")->value;
    env->qti_white_jitter = dict_get(kwargs, "qti_white_jitter")->value;
    env->qti_black_jitter = dict_get(kwargs, "qti_black_jitter")->value;
    env->qti_timeout = dict_get(kwargs, "qti_timeout")->value;

    env->line_width_m = dict_get(kwargs, "line_width_m")->value;
    env->line_width_jitter_m = dict_get(kwargs, "line_width_jitter_m")->value;
    env->line_edge_softness_m = dict_get(kwargs, "line_edge_softness_m")->value;
    env->line_edge_softness_jitter_m = dict_get(kwargs, "line_edge_softness_jitter_m")->value;
    env->track_bounds_m = dict_get(kwargs, "track_bounds_m")->value;
    env->start_lateral_offset_m = dict_get(kwargs, "start_lateral_offset_m")->value;
    env->start_heading_offset_rad = dict_get(kwargs, "start_heading_offset_rad")->value;

    apply_model_timing(env);
    init(env);
}

void my_log(Log* log, Dict* out) {
    dict_set(out, "perf", log->perf);
    dict_set(out, "score", log->score);
    dict_set(out, "progress_frac", log->progress_frac);
    dict_set(out, "distance_progress_frac", log->distance_progress_frac);
    dict_set(out, "progress_m", log->progress_m);
    dict_set(out, "episode_return", log->episode_return);
    dict_set(out, "episode_length", log->episode_length);
    dict_set(out, "centerline_error", log->centerline_error);
    dict_set(out, "accuracy", log->accuracy);
    dict_set(out, "checkpoint_accuracy", log->checkpoint_accuracy);
    dict_set(out, "checkpoints", log->checkpoints);
    dict_set(out, "target_checkpoints", log->target_checkpoints);
    dict_set(out, "progress_target_m", log->progress_target_m);
    dict_set(out, "avg_forward_speed_mps", log->avg_forward_speed_mps);
    dict_set(out, "avg_speed_frac", log->avg_speed_frac);
    dict_set(out, "idle_frac", log->idle_frac);
    dict_set(out, "turn_outer_speed_frac", log->turn_outer_speed_frac);
    dict_set(out, "negative_action_frac", log->negative_action_frac);
    dict_set(out, "action_bound_violation", log->action_bound_violation);
    dict_set(out, "raw_action_abs", log->raw_action_abs);
    dict_set(out, "terminal_timeout", log->terminal_timeout);
    dict_set(out, "terminal_lost_line", log->terminal_lost_line);
    dict_set(out, "terminal_too_far", log->terminal_too_far);
    dict_set(out, "terminal_track_complete", log->terminal_track_complete);
    dict_set(out, "terminal_negative", log->terminal_negative);
    dict_set(out, "terminal_action_bound", log->terminal_action_bound);
    dict_set(out, "terminal_success", log->terminal_success);
    dict_set(out, "n", log->n);
}
