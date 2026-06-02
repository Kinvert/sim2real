#ifndef LINE_FOLLOW_PUFFERNET_H
#define LINE_FOLLOW_PUFFERNET_H

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Weights {
    float* data;
    int raw_size;
    int size;
    int idx;
} Weights;

static int puffernet_align8(int value) {
    return (value + 7) & ~7;
}

static Weights* load_weights(const char* filename) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        perror("Error opening file");
        return NULL;
    }
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    rewind(file);
    size_t num_weights = (size_t)file_size / sizeof(float);
    Weights* weights = (Weights*)calloc(1,
        sizeof(Weights) + (num_weights + 64) * sizeof(float));
    weights->data = (float*)(weights + 1);
    size_t read_size = fread(weights->data, sizeof(float), num_weights, file);
    fclose(file);
    if (read_size != num_weights) {
        perror("Error reading file");
    }
    weights->raw_size = (int)num_weights;
    weights->size = (int)num_weights + 64;
    weights->idx = 0;
    return weights;
}

static void free_weights(Weights* weights) {
    if (weights == NULL) {
        return;
    }
    free(weights);
}

static int puffernet_storage_weight_count(int input_dim, int hidden_dim,
        int num_layers, int num_actions) {
    int decoder_rows = num_actions + 1;
    int index = 0;
    index = puffernet_align8(index);
    index += hidden_dim * input_dim;
    index = puffernet_align8(index);
    index += decoder_rows * hidden_dim;
    index = puffernet_align8(index);
    index += num_actions;
    for (int layer = 0; layer < num_layers; layer++) {
        index = puffernet_align8(index);
        index += 3 * hidden_dim * hidden_dim;
    }
    return index;
}

static int puffernet_aligned_read_weight_count(int input_dim, int hidden_dim,
        int num_layers, int num_actions) {
    return puffernet_align8(
        puffernet_storage_weight_count(input_dim, hidden_dim, num_layers, num_actions));
}

static float* get_weights_aligned(Weights* weights, int num_weights) {
    float* data = &weights->data[weights->idx];
    weights->idx += num_weights;
    weights->idx = puffernet_align8(weights->idx);
    assert(weights->idx <= weights->size);
    return data;
}

static inline float sigmoidf_local(float x) {
    return 1.0f / (1.0f + expf(-x));
}

typedef struct Linear {
    float* output;
    float* weights;
    int batch_size;
    int input_dim;
    int output_dim;
} Linear;

static Linear* make_linear(Weights* weights, int batch_size, int input_dim, int output_dim) {
    size_t buffer_size = (size_t)batch_size * (size_t)output_dim * sizeof(float);
    Linear* layer = (Linear*)calloc(1, sizeof(Linear) + buffer_size);
    *layer = (Linear){
        .output = (float*)(layer + 1),
        .weights = get_weights_aligned(weights, output_dim * input_dim),
        .batch_size = batch_size,
        .input_dim = input_dim,
        .output_dim = output_dim,
    };
    return layer;
}

static void linear(Linear* layer, float* input) {
    for (int b = 0; b < layer->batch_size; b++) {
        for (int o = 0; o < layer->output_dim; o++) {
            float sum = 0.0f;
            for (int i = 0; i < layer->input_dim; i++) {
                sum += input[b * layer->input_dim + i]
                    * layer->weights[o * layer->input_dim + i];
            }
            layer->output[b * layer->output_dim + o] = sum;
        }
    }
}

typedef struct MinGRU {
    float* state;
    float* output;
    Linear** proj;
    int batch_size;
    int hidden_size;
    int num_layers;
} MinGRU;

static MinGRU* make_mingru(Weights* weights, int batch_size, int hidden_size, int num_layers) {
    MinGRU* layer = (MinGRU*)calloc(1, sizeof(MinGRU));
    layer->state = (float*)calloc(
        (size_t)num_layers * (size_t)batch_size * (size_t)hidden_size, sizeof(float));
    layer->output = (float*)calloc((size_t)batch_size * (size_t)hidden_size, sizeof(float));
    layer->proj = (Linear**)calloc((size_t)num_layers, sizeof(Linear*));
    layer->batch_size = batch_size;
    layer->hidden_size = hidden_size;
    layer->num_layers = num_layers;
    for (int l = 0; l < num_layers; l++) {
        layer->proj[l] = make_linear(weights, batch_size, hidden_size, 3 * hidden_size);
    }
    return layer;
}

static void mingru(MinGRU* layer, float* input) {
    int B = layer->batch_size;
    int H = layer->hidden_size;
    if (layer->num_layers == 0) {
        memcpy(layer->output, input, (size_t)B * (size_t)H * sizeof(float));
        return;
    }

    float* x = input;
    for (int l = 0; l < layer->num_layers; l++) {
        float* state_l = layer->state + (size_t)l * (size_t)B * (size_t)H;
        linear(layer->proj[l], x);
        float* combined = layer->proj[l]->output;
        for (int b = 0; b < B; b++) {
            float* cb = combined + (size_t)b * (size_t)3 * (size_t)H;
            float* sb = state_l + (size_t)b * (size_t)H;
            float* xb = x + (size_t)b * (size_t)H;
            float* ob = layer->output + (size_t)b * (size_t)H;
            for (int h = 0; h < H; h++) {
                float hidden = cb[h];
                float gate = cb[H + h];
                float hw = cb[2 * H + h];
                float s = sb[h];
                float gate_s = sigmoidf_local(gate);
                float h_tilde = hidden >= 0.0f ? hidden + 0.5f : sigmoidf_local(hidden);
                float mingru_out = s + gate_s * (h_tilde - s);
                float hw_s = sigmoidf_local(hw);
                ob[h] = hw_s * mingru_out + (1.0f - hw_s) * xb[h];
                sb[h] = mingru_out;
            }
        }
        x = layer->output;
    }
}

static void free_mingru(MinGRU* layer) {
    for (int l = 0; l < layer->num_layers; l++) {
        free(layer->proj[l]);
    }
    free(layer->state);
    free(layer->output);
    free(layer->proj);
    free(layer);
}

typedef struct PufferNet {
    int num_agents;
    float* obs;
    Linear* encoder;
    MinGRU* mingru;
    Linear* decoder;
    float* log_std;
    int is_continuous;
    int num_actions;
} PufferNet;

static PufferNet* make_puffernet(Weights* weights, int num_agents, int input_dim,
        int hidden_dim, int num_layers, int logit_sizes[], int num_actions) {
    PufferNet* net = (PufferNet*)calloc(1, sizeof(PufferNet));
    net->num_agents = num_agents;
    net->obs = (float*)calloc((size_t)num_agents * (size_t)input_dim, sizeof(float));
    net->is_continuous = 1;
    net->num_actions = num_actions;
    for (int i = 0; i < num_actions; i++) {
        if (logit_sizes[i] != 1) {
            net->is_continuous = 0;
        }
    }
    assert(net->is_continuous && "line_follow host smoke only supports continuous policies");
    int storage_size = puffernet_storage_weight_count(
        input_dim, hidden_dim, num_layers, num_actions);
    int aligned_read_size = puffernet_aligned_read_weight_count(
        input_dim, hidden_dim, num_layers, num_actions);
    if (weights->raw_size != storage_size && weights->raw_size != aligned_read_size) {
        fprintf(stderr,
            "Checkpoint weight layout does not match model shape: raw_floats=%d storage=%d aligned_read=%d hidden_size=%d num_layers=%d\n",
            weights->raw_size,
            storage_size,
            aligned_read_size,
            hidden_dim,
            num_layers);
        assert(false && "checkpoint weight layout does not match model shape");
        abort();
    }
    weights->idx = 0;

    net->encoder = make_linear(weights, num_agents, input_dim, hidden_dim);
    net->decoder = make_linear(weights, num_agents, hidden_dim, num_actions + 1);
    net->log_std = get_weights_aligned(weights, num_actions);
    net->mingru = make_mingru(weights, num_agents, hidden_dim, num_layers);
    return net;
}

static void forward_puffernet(PufferNet* net, float* observations, float* actions) {
    linear(net->encoder, observations);
    mingru(net->mingru, net->encoder->output);
    linear(net->decoder, net->mingru->output);
    for (int b = 0; b < net->num_agents; b++) {
        int in_adr = b * (net->num_actions + 1);
        for (int a = 0; a < net->num_actions; a++) {
            actions[b * net->num_actions + a] = net->decoder->output[in_adr + a];
        }
    }
}

static void free_puffernet(PufferNet* net) {
    free(net->obs);
    free(net->encoder);
    free(net->decoder);
    free_mingru(net->mingru);
    free(net);
}

#endif
