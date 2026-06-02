#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "line_follow.h"

static void set_command_actions(LineFollow* env, float left, float right) {
    env->actions[0] = raw_action_from_command(left);
    env->actions[1] = raw_action_from_command(right);
}

static void set_heuristic_actions(LineFollow* env) {
    float left_dark = env->observations[0];
    float right_dark = env->observations[2];
    float correction = 0.85f * (left_dark - right_dark);
    float base = 0.42f;

    bool line_seen = false;
    for (int i = 0; i < LINE_FOLLOW_OBS_SIZE; i++) {
        if (env->observations[i] >= env->qti_threshold) {
            line_seen = true;
        }
    }

    if (!line_seen) {
        set_command_actions(env, base, base);
        return;
    }

    set_command_actions(env,
        clampf(base - correction, -1.0f, 1.0f),
        clampf(base + correction, -1.0f, 1.0f));
}

static void set_manual_actions(LineFollow* env) {
    float throttle = 0.0f;
    float turn = 0.0f;
    if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) throttle += 0.7f;
    if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) throttle -= 0.7f;
    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) turn -= 0.55f;
    if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) turn += 0.55f;

    set_command_actions(env,
        clampf(throttle - turn, -1.0f, 1.0f),
        clampf(throttle + turn, -1.0f, 1.0f));
}

int main(int argc, char** argv) {
    LineFollow env;
    memset(&env, 0, sizeof(env));
    set_defaults(&env);
    apply_model_timing(&env);
    env.rng = (unsigned int)time(NULL);

    bool manual = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--manual") == 0) {
            manual = true;
        } else if (strcmp(argv[i], "--family") == 0 && i + 1 < argc) {
            env.track_family = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: ./line_follow [--manual] [--family -1|0|1|2|3|4]\n");
            printf("Controls: H toggles heuristic/manual, WASD or arrows drive in manual mode.\n");
            return 0;
        }
    }

    float observations[LINE_FOLLOW_OBS_SIZE] = {0};
    float actions[LINE_FOLLOW_NUM_ATNS] = {0};
    float rewards[1] = {0};
    float terminals[1] = {0};
    env.observations = observations;
    env.actions = actions;
    env.rewards = rewards;
    env.terminals = terminals;

    init(&env);
    c_reset(&env);
    c_render(&env);

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_H)) {
            manual = !manual;
        }

        if (manual) {
            set_manual_actions(&env);
        } else {
            set_heuristic_actions(&env);
        }

        c_step(&env);
        c_render(&env);
    }

    c_close(&env);
    return 0;
}
