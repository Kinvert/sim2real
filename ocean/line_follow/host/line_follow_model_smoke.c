#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ocean/line_follow/host/line_follow_puffernet.h"

#define OBS_SIZE 3
#define NUM_ACTIONS 2
#ifndef HIDDEN_SIZE
#define HIDDEN_SIZE 4
#endif
#ifndef NUM_LAYERS
#define NUM_LAYERS 0
#endif
#ifndef MAX_WHEEL_SPEED_MPS
#define MAX_WHEEL_SPEED_MPS 0.038f
#endif
#define COMMAND_DEADBAND 0.04f
#define TIRE_DIAMETER_M 0.065f
#define TICKS_PER_REV 64.0f

typedef struct {
    const char* name;
    float obs[OBS_SIZE];
} Case;

static float clampf_local(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float bounded_policy_action(float raw_action) {
    return clampf_local(raw_action, -1.0f, 1.0f);
}

static float command_to_mps(float action) {
    float clipped = clampf_local(action, -1.0f, 1.0f);
    float unit = clipped < 0.0f ? 0.0f : clipped;
    if (unit < COMMAND_DEADBAND) {
        unit = 0.0f;
    }
    return unit * MAX_WHEEL_SPEED_MPS;
}

static int mps_to_ticks(float mps) {
    float circumference_m = (float)M_PI * TIRE_DIAMETER_M;
    float ticks = (mps / circumference_m) * TICKS_PER_REV;
    return (int)lrintf(ticks);
}

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

static void run_case(PufferNet* net, const char* name, const float obs[OBS_SIZE]) {
    float actions[NUM_ACTIONS] = {0.0f, 0.0f};
    reset_recurrent_state(net);
    forward_puffernet(net, (float*)obs, actions);
    actions[0] = bounded_policy_action(actions[0]);
    actions[1] = bounded_policy_action(actions[1]);

    float left_mps = command_to_mps(actions[0]);
    float right_mps = command_to_mps(actions[1]);
    int left_ticks = mps_to_ticks(left_mps);
    int right_ticks = mps_to_ticks(right_mps);

    printf("%-18s obs=[%.2f %.2f %.2f]  action=[% .3f % .3f]"
           "  mps=[% .3f % .3f]  ticks/s=[% 4d % 4d]\n",
        name, obs[0], obs[1], obs[2],
        actions[0], actions[1], left_mps, right_mps, left_ticks, right_ticks);
}

int main(int argc, char** argv) {
    if (argc != 2 && argc != 5) {
        fprintf(stderr, "Usage: %s WEIGHTS.bin [left middle right]\n", argv[0]);
        return 2;
    }

    const char* weights_path = argv[1];
    Weights* weights = load_weights(weights_path);
    if (weights == NULL) {
        return 1;
    }

    int raw_floats = weights->size - 7;
    int expected = expected_raw_float_count();
    if (raw_floats != expected) {
        fprintf(stderr,
            "Checkpoint size does not match compiled model shape: raw_floats=%d expected=%d hidden_size=%d num_layers=%d\n",
            raw_floats, expected, HIDDEN_SIZE, NUM_LAYERS);
        free(weights);
        return 1;
    }

    int logit_sizes[NUM_ACTIONS] = {1, 1};
    PufferNet* net = make_puffernet(weights, 1, OBS_SIZE, HIDDEN_SIZE,
        NUM_LAYERS, logit_sizes, NUM_ACTIONS);

    printf("weights=%s bytes=%ld floats=%d\n", weights_path,
        (long)raw_floats * (long)sizeof(float), raw_floats);
    printf("scale: action <= 0 stops wheel, action 1 -> %.3f m/s -> approx %.1f ticks/s"
           " with %.0fmm tire and %.0f ticks/rev\n",
        MAX_WHEEL_SPEED_MPS,
        (MAX_WHEEL_SPEED_MPS / ((float)M_PI * TIRE_DIAMETER_M)) * TICKS_PER_REV,
        TIRE_DIAMETER_M * 1000.0f, TICKS_PER_REV);
    printf("note: each row resets the MinGRU state, so cases are independent.\n\n");

    if (argc == 5) {
        float obs[OBS_SIZE];
        for (int i = 0; i < OBS_SIZE; i++) {
            obs[i] = strtof(argv[i + 2], NULL);
        }
        run_case(net, "custom", obs);
    } else {
        const Case cases[] = {
            {"all_white", {0.0f, 0.0f, 0.0f}},
            {"all_black", {1.0f, 1.0f, 1.0f}},
            {"left", {1.0f, 0.0f, 0.0f}},
            {"middle", {0.0f, 1.0f, 0.0f}},
            {"right", {0.0f, 0.0f, 1.0f}},
            {"soft_left", {0.8f, 0.2f, 0.0f}},
            {"soft_right", {0.0f, 0.2f, 0.8f}},
            {"balanced_edge", {0.2f, 1.0f, 0.2f}},
        };
        int num_cases = (int)(sizeof(cases) / sizeof(cases[0]));
        for (int i = 0; i < num_cases; i++) {
            run_case(net, cases[i].name, cases[i].obs);
        }
    }

    free_puffernet(net);
    free(weights);
    return 0;
}
