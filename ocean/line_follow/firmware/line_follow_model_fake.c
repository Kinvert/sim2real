/*
  Line-follow trained-model fake-observation smoke test.

  This runs on the Propeller. It embeds the trained PufferLib native checkpoint
  weights, injects canned normalized QTI observations, runs the policy forward
  pass in Propeller C, and prints the resulting wheel commands. It never reads
  QTI pins and never commands motors.
*/

#include "simpletools.h"
#include <math.h>

#include "line_follow_model_weights.h"

#define OBS_SIZE 4
#ifndef HIDDEN_SIZE
#define HIDDEN_SIZE 8
#endif
#ifndef NUM_LAYERS
#define NUM_LAYERS 0
#endif
#define NUM_ACTIONS 2
#define DECODER_SIZE 3
#ifndef MAX_WHEEL_SPEED_MPS
#define MAX_WHEEL_SPEED_MPS 0.010f
#endif
#define COMMAND_DEADBAND 0.04f
#define TIRE_DIAMETER_M 0.065f
#define TICKS_PER_REV 64.0f
#define LINE_FOLLOW_PI 3.14159265358979323846f

typedef struct {
  const char* name;
  int obs_q1000[OBS_SIZE];
} FakeCase;

static const float* encoder_w;
static const float* decoder_w;
static const float* mingru_w;
static int weight_idx;

static float state[HIDDEN_SIZE];
static float encoder_out[HIDDEN_SIZE];
static float combined[3 * HIDDEN_SIZE];
static float mingru_out[HIDDEN_SIZE];
static float decoder_out[DECODER_SIZE];

static int expected_raw_float_count(void)
{
  return HIDDEN_SIZE * OBS_SIZE
    + DECODER_SIZE * HIDDEN_SIZE
    + NUM_ACTIONS
    + NUM_LAYERS * 3 * HIDDEN_SIZE * HIDDEN_SIZE;
}

static const float* take_aligned_weights(int count)
{
  const float* out = &line_follow_model_weights[weight_idx];
  weight_idx += count;
  weight_idx = (weight_idx + 7) & ~7;
  return out;
}

static void init_model(void)
{
  weight_idx = 0;
  encoder_w = take_aligned_weights(HIDDEN_SIZE * OBS_SIZE);
  decoder_w = take_aligned_weights(DECODER_SIZE * HIDDEN_SIZE);
  (void)take_aligned_weights(NUM_ACTIONS);
#if NUM_LAYERS > 0
  mingru_w = take_aligned_weights(3 * HIDDEN_SIZE * HIDDEN_SIZE);
#endif
}

static void reset_state(void)
{
  int i;
  for(i = 0; i < HIDDEN_SIZE; i++)
  {
    state[i] = 0.0f;
  }
}

static float sigmoidf_model(float x)
{
  return 1.0f / (1.0f + expf(-x));
}

static float clampf_model(float value, float lo, float hi)
{
  if(value < lo) return lo;
  if(value > hi) return hi;
  return value;
}

static void linear_forward(const float* input, const float* weights,
    float* output, int input_dim, int output_dim)
{
  int o;
  for(o = 0; o < output_dim; o++)
  {
    float sum = 0.0f;
    int i;
    for(i = 0; i < input_dim; i++)
    {
      sum += input[i] * weights[o * input_dim + i];
    }
    output[o] = sum;
  }
}

static void forward_model(const float obs[OBS_SIZE], float actions[NUM_ACTIONS])
{
  int h;

  linear_forward(obs, encoder_w, encoder_out, OBS_SIZE, HIDDEN_SIZE);
#if NUM_LAYERS > 0
  linear_forward(encoder_out, mingru_w, combined, HIDDEN_SIZE, 3 * HIDDEN_SIZE);

  for(h = 0; h < HIDDEN_SIZE; h++)
  {
    float hidden = combined[h];
    float gate = combined[HIDDEN_SIZE + h];
    float highway = combined[2 * HIDDEN_SIZE + h];
    float previous = state[h];
    float gate_s = sigmoidf_model(gate);
    float hidden_tilde = hidden >= 0.0f ? hidden + 0.5f : sigmoidf_model(hidden);
    float next_state = previous + gate_s * (hidden_tilde - previous);
    float highway_s = sigmoidf_model(highway);
    mingru_out[h] = highway_s * next_state + (1.0f - highway_s) * encoder_out[h];
    state[h] = next_state;
  }

  linear_forward(mingru_out, decoder_w, decoder_out, HIDDEN_SIZE, DECODER_SIZE);
#else
  (void)h;
  linear_forward(encoder_out, decoder_w, decoder_out, HIDDEN_SIZE, DECODER_SIZE);
#endif
  actions[0] = decoder_out[0];
  actions[1] = decoder_out[1];
}

static int action_to_ticks(float action)
{
  float clipped = clampf_model(action, -1.0f, 1.0f);
  float mps;
  float ticks;

  if(clipped < 0.0f)
  {
    clipped = 0.0f;
  }

  if(fabsf(clipped) < COMMAND_DEADBAND)
  {
    clipped = 0.0f;
  }

  mps = clipped * MAX_WHEEL_SPEED_MPS;
  ticks = (mps / (LINE_FOLLOW_PI * TIRE_DIAMETER_M)) * TICKS_PER_REV;
  if(ticks >= 0.0f)
  {
    return (int)(ticks + 0.5f);
  }
  return (int)(ticks - 0.5f);
}

static void print_q1000(int value)
{
  int clipped = value;
  if(clipped < 0) clipped = 0;
  if(clipped > 1000) clipped = 1000;
  print("%d.%03d", clipped / 1000, clipped % 1000);
}

static void print_float3(float value)
{
  int scaled;
  if(value >= 0.0f)
  {
    scaled = (int)(value * 1000.0f + 0.5f);
    print(" ");
  }
  else
  {
    scaled = (int)(value * -1000.0f + 0.5f);
    print("-");
  }
  print("%d.%03d", scaled / 1000, scaled % 1000);
}

static void run_case(const FakeCase* test_case)
{
  float obs[OBS_SIZE];
  float actions[NUM_ACTIONS];
  int left_ticks;
  int right_ticks;
  int i;

  for(i = 0; i < OBS_SIZE; i++)
  {
    obs[i] = (float)test_case->obs_q1000[i] / 1000.0f;
  }

  reset_state();
  forward_model(obs, actions);
  left_ticks = action_to_ticks(actions[0]);
  right_ticks = action_to_ticks(actions[1]);

  print("M %s ", test_case->name);
  print_q1000(test_case->obs_q1000[0]);
  print(" ");
  print_q1000(test_case->obs_q1000[1]);
  print(" ");
  print_q1000(test_case->obs_q1000[2]);
  print(" ");
  print_q1000(test_case->obs_q1000[3]);
  print(" ");
  print_float3(actions[0]);
  print(" ");
  print_float3(actions[1]);
  print(" %d %d\n", left_ticks, right_ticks);
}

int main(void)
{
  int i;
  FakeCase cases[] = {
    {"all_white",    {0,    0,    0,    0}},
    {"all_black",    {1000, 1000, 1000, 1000}},
    {"center_black", {0,    1000, 1000, 0}},
    {"left_black",   {1000, 1000, 0,    0}},
    {"right_black",  {0,    0,    1000, 1000}},
    {"outer_left",   {1000, 0,    0,    0}},
    {"outer_right",  {0,    0,    0,    1000}},
    {"inner_left",   {0,    1000, 0,    0}},
    {"inner_right",  {0,    0,    1000, 0}},
    {"soft_left",    {400,  800,  100,  0}},
    {"soft_right",   {0,    100,  800,  400}},
  };
  int num_cases = sizeof(cases) / sizeof(cases[0]);

  low(26);
  low(27);
  if(LINE_FOLLOW_MODEL_RAW_FLOATS != expected_raw_float_count())
  {
    print("\nLINE_FOLLOW_MODEL_FAKE shape mismatch\n");
    print("raw_floats=%d expected=%d hidden_size=%d num_layers=%d\n",
        LINE_FOLLOW_MODEL_RAW_FLOATS, expected_raw_float_count(), HIDDEN_SIZE, NUM_LAYERS);
    while(1)
    {
      pause(1000);
    }
  }
  init_model();

  print("\nLINE_FOLLOW_MODEL_FAKE v1\n");
  print("checkpoint=%s\n", line_follow_model_checkpoint);
  print("raw_floats=%d padded_floats=%d\n",
      LINE_FOLLOW_MODEL_RAW_FLOATS, LINE_FOLLOW_MODEL_PADDED_FLOATS);
  print("hidden_size=%d num_layers=%d\n", HIDDEN_SIZE, NUM_LAYERS);
  print("scale action[-1,1] -> +/-%d mm/s, tire=65 mm, ticks/rev=64\n",
      (int)(MAX_WHEEL_SPEED_MPS * 1000.0f + 0.5f));
  print("no QTI reads, no drive output, MinGRU state reset per row\n");
  print("M name obs0 obs1 obs2 obs3 action0 action1 left right\n");

  for(i = 0; i < num_cases; i++)
  {
    run_case(&cases[i]);
  }

  print("LINE_FOLLOW_MODEL_FAKE done\n");
  while(1)
  {
    pause(1000);
  }
}
