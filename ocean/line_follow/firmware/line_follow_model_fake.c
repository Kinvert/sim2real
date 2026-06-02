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

#define OBS_SIZE 3
#ifndef HIDDEN_SIZE
#define HIDDEN_SIZE 4
#endif
#ifndef NUM_LAYERS
#define NUM_LAYERS 0
#endif
#define NUM_ACTIONS 2
#define DECODER_SIZE 3
#ifndef MAX_WHEEL_SPEED_MPS
#define MAX_WHEEL_SPEED_MPS 0.116f
#endif
#ifndef COMMAND_DEADBAND
#define COMMAND_DEADBAND 0.04f
#endif
#ifndef MIN_DRIVE_TICKS_PER_SEC
#define MIN_DRIVE_TICKS_PER_SEC 6
#endif
#define TIRE_DIAMETER_M 0.065f
#define TICKS_PER_REV 64.0f
#define LINE_FOLLOW_PI 3.14159265358979323846f
#ifndef COMMAND_DEADBAND_Q1000
#define COMMAND_DEADBAND_Q1000 ((int)(COMMAND_DEADBAND * 1000.0f + 0.5f))
#endif
#ifndef MAX_WHEEL_TICKS_PER_SEC_Q1000
#define MAX_WHEEL_TICKS_PER_SEC_Q1000 ((int)(((MAX_WHEEL_SPEED_MPS / (LINE_FOLLOW_PI * TIRE_DIAMETER_M)) * TICKS_PER_REV * 1000.0f) + 0.5f))
#endif

#define LINE_FOLLOW_INFERENCE_FULL 0
#define LINE_FOLLOW_INFERENCE_FOLDED_FLOAT 1
#define LINE_FOLLOW_INFERENCE_FIXED 2
#ifndef LINE_FOLLOW_INFERENCE_MODE
#define LINE_FOLLOW_INFERENCE_MODE LINE_FOLLOW_INFERENCE_FIXED
#endif

#if LINE_FOLLOW_INFERENCE_MODE < LINE_FOLLOW_INFERENCE_FULL || LINE_FOLLOW_INFERENCE_MODE > LINE_FOLLOW_INFERENCE_FIXED
#error "LINE_FOLLOW_INFERENCE_MODE must be 0=full, 1=folded-float, or 2=fixed"
#endif
#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FOLDED_FLOAT && !(NUM_LAYERS == 0 && LINE_FOLLOW_MODEL_FOLDED_SUPPORTED)
#error "folded inference requires NUM_LAYERS=0 and a folded model header"
#endif

#ifndef LINE_FOLLOW_FAKE_REPLAY
#define LINE_FOLLOW_FAKE_REPLAY 0
#endif

#ifndef LINE_FOLLOW_FAKE_RESET_EACH_ROW
#define LINE_FOLLOW_FAKE_RESET_EACH_ROW 1
#endif

typedef struct {
  const char* name;
  int obs_q1000[OBS_SIZE];
} FakeCase;

static const float* encoder_w;
static const float* decoder_w;
static const float* mingru_w;
static const int* encoder_w_q;
static const int* decoder_w_q;
static const int* mingru_w_q;
static int weight_idx;
static int weight_q_idx;

static float state[HIDDEN_SIZE];
static float encoder_out[HIDDEN_SIZE];
static float combined[3 * HIDDEN_SIZE];
static float mingru_out[HIDDEN_SIZE];
static float decoder_out[DECODER_SIZE];
static int state_q[HIDDEN_SIZE];
static int encoder_out_q[HIDDEN_SIZE];
static int combined_q[3 * HIDDEN_SIZE];
static int mingru_out_q[HIDDEN_SIZE];
static int decoder_out_q[DECODER_SIZE];

static int align8_int(int value)
{
  return (value + 7) & ~7;
}

static int expected_raw_float_count(void)
{
  int index = 0;
  int layer;
  index = align8_int(index);
  index += HIDDEN_SIZE * OBS_SIZE;
  index = align8_int(index);
  index += DECODER_SIZE * HIDDEN_SIZE;
  index = align8_int(index);
  index += NUM_ACTIONS;
  for(layer = 0; layer < NUM_LAYERS; layer++)
  {
    index = align8_int(index);
    index += 3 * HIDDEN_SIZE * HIDDEN_SIZE;
  }
  return index;
}

static int expected_aligned_read_float_count(void)
{
  return align8_int(expected_raw_float_count());
}

static const float* take_aligned_weights(int count)
{
  const float* out = &line_follow_model_weights[weight_idx];
  weight_idx += count;
  weight_idx = align8_int(weight_idx);
  return out;
}

static const int* take_aligned_weights_q(int count)
{
  const int* out = &line_follow_model_weights_q[weight_q_idx];
  weight_q_idx += count;
  weight_q_idx = align8_int(weight_q_idx);
  return out;
}

static void init_model(void)
{
  weight_idx = 0;
  weight_q_idx = 0;
#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FULL
  encoder_w = take_aligned_weights(HIDDEN_SIZE * OBS_SIZE);
  decoder_w = take_aligned_weights(DECODER_SIZE * HIDDEN_SIZE);
  (void)take_aligned_weights(NUM_ACTIONS);
#if NUM_LAYERS > 0
  mingru_w = take_aligned_weights(3 * HIDDEN_SIZE * HIDDEN_SIZE);
#endif
#endif
#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FIXED
  encoder_w_q = take_aligned_weights_q(HIDDEN_SIZE * OBS_SIZE);
  decoder_w_q = take_aligned_weights_q(DECODER_SIZE * HIDDEN_SIZE);
  (void)take_aligned_weights_q(NUM_ACTIONS);
#if NUM_LAYERS > 0
  mingru_w_q = take_aligned_weights_q(3 * HIDDEN_SIZE * HIDDEN_SIZE);
#endif
#endif
}

static void reset_state(void)
{
  int i;
  for(i = 0; i < HIDDEN_SIZE; i++)
  {
    state[i] = 0.0f;
    state_q[i] = 0;
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

static int clamp_int(int value, int lo, int hi)
{
  if(value < lo) return lo;
  if(value > hi) return hi;
  return value;
}

static int round_div_int(int numerator, int denominator)
{
  if(numerator >= 0)
  {
    return (numerator + denominator / 2) / denominator;
  }
  return -((-numerator + denominator / 2) / denominator);
}

static int round_div_ll(long long numerator, int denominator)
{
  if(numerator >= 0)
  {
    return (int)((numerator + denominator / 2) / denominator);
  }
  return (int)(-((-numerator + denominator / 2) / denominator));
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

#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FOLDED_FLOAT
  (void)h;
  linear_forward(obs, line_follow_model_folded_action_w,
      actions, OBS_SIZE, NUM_ACTIONS);
#else
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
  actions[0] = clampf_model(decoder_out[0], -1.0f, 1.0f);
  actions[1] = clampf_model(decoder_out[1], -1.0f, 1.0f);
#endif
  actions[0] = clampf_model(actions[0], -1.0f, 1.0f);
  actions[1] = clampf_model(actions[1], -1.0f, 1.0f);
}

static void forward_model_fixed(const int obs_q1000[OBS_SIZE],
    int action_fixed[NUM_ACTIONS], float actions[NUM_ACTIONS])
{
#define MODEL_INTERNAL_LIMIT (LINE_FOLLOW_MODEL_FIXED_SCALE * 48)
#define MODEL_SIGMOID_LUT_MAX_INDEX 128
#define MODEL_SIGMOID_LUT_STEP (LINE_FOLLOW_MODEL_FIXED_SCALE / 8)
  static const int sigmoid_lut_q[MODEL_SIGMOID_LUT_MAX_INDEX + 1] = {
    5, 6, 7, 8, 9, 10, 12, 13,
    15, 17, 19, 22, 25, 28, 32, 36,
    41, 46, 52, 59, 67, 76, 86, 97,
    110, 124, 141, 159, 180, 204, 230, 261,
    295, 333, 376, 425, 480, 542, 612, 690,
    777, 875, 984, 1107, 1243, 1394, 1562, 1748,
    1953, 2178, 2426, 2695, 2989, 3307, 3649, 4015,
    4406, 4820, 5256, 5712, 6186, 6674, 7173, 7681,
    8192, 8703, 9211, 9710, 10198, 10672, 11128, 11564,
    11978, 12369, 12735, 13077, 13395, 13689, 13958, 14206,
    14431, 14636, 14822, 14990, 15141, 15277, 15400, 15509,
    15607, 15694, 15772, 15842, 15904, 15959, 16008, 16051,
    16089, 16123, 16154, 16180, 16204, 16225, 16243, 16260,
    16274, 16287, 16298, 16308, 16317, 16325, 16332, 16338,
    16343, 16348, 16352, 16356, 16359, 16362, 16365, 16367,
    16369, 16371, 16372, 16374, 16375, 16376, 16377, 16378,
    16379
  };
  int action;
#if NUM_LAYERS == 0 && LINE_FOLLOW_MODEL_FOLDED_SUPPORTED
  for(action = 0; action < NUM_ACTIONS; action++)
  {
    int i;
    long long total = 0;
    for(i = 0; i < OBS_SIZE; i++)
    {
      total += (long long)obs_q1000[i]
        * line_follow_model_folded_action_w_q[action * OBS_SIZE + i];
    }
    action_fixed[action] = clamp_int(
        round_div_ll(total, 1000),
        -LINE_FOLLOW_MODEL_FIXED_SCALE, LINE_FOLLOW_MODEL_FIXED_SCALE);
    actions[action] = (float)action_fixed[action]
      / (float)LINE_FOLLOW_MODEL_FIXED_SCALE;
  }
#else
  int h;
  int o;
  int step = MODEL_SIGMOID_LUT_STEP;

  for(o = 0; o < HIDDEN_SIZE; o++)
  {
    int i;
    long long total = 0;
    for(i = 0; i < OBS_SIZE; i++)
    {
      total += (long long)obs_q1000[i] * encoder_w_q[o * OBS_SIZE + i];
    }
    encoder_out_q[o] = clamp_int(round_div_ll(total, 1000),
        -MODEL_INTERNAL_LIMIT, MODEL_INTERNAL_LIMIT);
  }

#if NUM_LAYERS > 0
  for(o = 0; o < 3 * HIDDEN_SIZE; o++)
  {
    int i;
    long long total = 0;
    for(i = 0; i < HIDDEN_SIZE; i++)
    {
      total += (long long)encoder_out_q[i] * mingru_w_q[o * HIDDEN_SIZE + i];
    }
    combined_q[o] = clamp_int(round_div_ll(total, LINE_FOLLOW_MODEL_FIXED_SCALE),
        -MODEL_INTERNAL_LIMIT, MODEL_INTERNAL_LIMIT);
  }

  for(h = 0; h < HIDDEN_SIZE; h++)
  {
    int hidden = combined_q[h];
    int gate = combined_q[HIDDEN_SIZE + h];
    int highway = combined_q[2 * HIDDEN_SIZE + h];
    int gate_s;
    int highway_s;
    int hidden_tilde;
    int next_state;
    int shifted;
    int idx;
    int frac;

    if(gate <= -8 * LINE_FOLLOW_MODEL_FIXED_SCALE) gate_s = sigmoid_lut_q[0];
    else if(gate >= 8 * LINE_FOLLOW_MODEL_FIXED_SCALE) gate_s = sigmoid_lut_q[MODEL_SIGMOID_LUT_MAX_INDEX];
    else
    {
      shifted = gate + 8 * LINE_FOLLOW_MODEL_FIXED_SCALE;
      idx = shifted / step;
      frac = shifted - idx * step;
      gate_s = sigmoid_lut_q[idx] + round_div_ll(
          (long long)(sigmoid_lut_q[idx + 1] - sigmoid_lut_q[idx]) * frac, step);
    }

    if(highway <= -8 * LINE_FOLLOW_MODEL_FIXED_SCALE) highway_s = sigmoid_lut_q[0];
    else if(highway >= 8 * LINE_FOLLOW_MODEL_FIXED_SCALE) highway_s = sigmoid_lut_q[MODEL_SIGMOID_LUT_MAX_INDEX];
    else
    {
      shifted = highway + 8 * LINE_FOLLOW_MODEL_FIXED_SCALE;
      idx = shifted / step;
      frac = shifted - idx * step;
      highway_s = sigmoid_lut_q[idx] + round_div_ll(
          (long long)(sigmoid_lut_q[idx + 1] - sigmoid_lut_q[idx]) * frac, step);
    }

    if(hidden >= 0)
    {
      hidden_tilde = hidden + LINE_FOLLOW_MODEL_FIXED_SCALE / 2;
    }
    else if(hidden <= -8 * LINE_FOLLOW_MODEL_FIXED_SCALE) hidden_tilde = sigmoid_lut_q[0];
    else
    {
      shifted = hidden + 8 * LINE_FOLLOW_MODEL_FIXED_SCALE;
      idx = shifted / step;
      frac = shifted - idx * step;
      hidden_tilde = sigmoid_lut_q[idx] + round_div_ll(
          (long long)(sigmoid_lut_q[idx + 1] - sigmoid_lut_q[idx]) * frac, step);
    }

    next_state = state_q[h] + round_div_ll(
        (long long)gate_s * (hidden_tilde - state_q[h]),
        LINE_FOLLOW_MODEL_FIXED_SCALE);
    next_state = clamp_int(next_state, -MODEL_INTERNAL_LIMIT, MODEL_INTERNAL_LIMIT);
    mingru_out_q[h] = round_div_ll(
        (long long)highway_s * next_state
        + (long long)(LINE_FOLLOW_MODEL_FIXED_SCALE - highway_s) * encoder_out_q[h],
        LINE_FOLLOW_MODEL_FIXED_SCALE);
    mingru_out_q[h] = clamp_int(mingru_out_q[h],
        -MODEL_INTERNAL_LIMIT, MODEL_INTERNAL_LIMIT);
    state_q[h] = next_state;
  }

  for(o = 0; o < DECODER_SIZE; o++)
  {
    int i;
    long long total = 0;
    for(i = 0; i < HIDDEN_SIZE; i++)
    {
      total += (long long)mingru_out_q[i] * decoder_w_q[o * HIDDEN_SIZE + i];
    }
    decoder_out_q[o] = clamp_int(round_div_ll(total, LINE_FOLLOW_MODEL_FIXED_SCALE),
        -MODEL_INTERNAL_LIMIT, MODEL_INTERNAL_LIMIT);
  }
#else
  for(o = 0; o < DECODER_SIZE; o++)
  {
    int i;
    long long total = 0;
    for(i = 0; i < HIDDEN_SIZE; i++)
    {
      total += (long long)encoder_out_q[i] * decoder_w_q[o * HIDDEN_SIZE + i];
    }
    decoder_out_q[o] = clamp_int(round_div_ll(total, LINE_FOLLOW_MODEL_FIXED_SCALE),
        -MODEL_INTERNAL_LIMIT, MODEL_INTERNAL_LIMIT);
  }
#endif

  for(action = 0; action < NUM_ACTIONS; action++)
  {
    action_fixed[action] = clamp_int(decoder_out_q[action],
        -LINE_FOLLOW_MODEL_FIXED_SCALE, LINE_FOLLOW_MODEL_FIXED_SCALE);
    actions[action] = (float)action_fixed[action]
      / (float)LINE_FOLLOW_MODEL_FIXED_SCALE;
  }
#endif
}

static int action_to_ticks(float action)
{
  float clipped = clampf_model(action, -1.0f, 1.0f);
  float unit = clipped;
  float mps;
  float ticks;
  int rounded;
  int min_ticks = MIN_DRIVE_TICKS_PER_SEC;
  int max_ticks;

  if(unit < 0.0f)
  {
    unit = 0.0f;
  }

  if(unit < COMMAND_DEADBAND)
  {
    unit = 0.0f;
  }

  mps = unit * MAX_WHEEL_SPEED_MPS;
  ticks = (mps / (LINE_FOLLOW_PI * TIRE_DIAMETER_M)) * TICKS_PER_REV;
  max_ticks = (int)(((MAX_WHEEL_SPEED_MPS / (LINE_FOLLOW_PI * TIRE_DIAMETER_M))
      * TICKS_PER_REV) + 0.5f);
  if(ticks >= 0.0f)
  {
    rounded = (int)(ticks + 0.5f);
  }
  else
  {
    rounded = (int)(ticks - 0.5f);
  }
  if(min_ticks < 0) min_ticks = 0;
  if(max_ticks < 0) max_ticks = 0;
  if(min_ticks > max_ticks) min_ticks = max_ticks;
  if(rounded < min_ticks) return min_ticks;
  return rounded;
}

static int action_fixed_to_ticks(int action_fixed)
{
  int unit = clamp_int(action_fixed,
      -LINE_FOLLOW_MODEL_FIXED_SCALE, LINE_FOLLOW_MODEL_FIXED_SCALE);
  int rounded;
  int min_ticks = MIN_DRIVE_TICKS_PER_SEC;
  int max_ticks = (MAX_WHEEL_TICKS_PER_SEC_Q1000 + 500) / 1000;

  if(unit < 0)
  {
    unit = 0;
  }

  if(unit < (COMMAND_DEADBAND_Q1000 * LINE_FOLLOW_MODEL_FIXED_SCALE + 500) / 1000)
  {
    unit = 0;
  }

  rounded = round_div_int(unit * MAX_WHEEL_TICKS_PER_SEC_Q1000,
      LINE_FOLLOW_MODEL_FIXED_SCALE * 1000);
  if(min_ticks < 0) min_ticks = 0;
  if(max_ticks < 0) max_ticks = 0;
  if(min_ticks > max_ticks) min_ticks = max_ticks;
  if(rounded < min_ticks) return min_ticks;
  return rounded;
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
  int action_fixed[NUM_ACTIONS] = {0, 0};
  int left_ticks;
  int right_ticks;
  int i;

#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FIXED
  (void)obs;
  if(LINE_FOLLOW_FAKE_RESET_EACH_ROW)
  {
    reset_state();
  }
  forward_model_fixed(test_case->obs_q1000, action_fixed, actions);
  left_ticks = action_fixed_to_ticks(action_fixed[0]);
  right_ticks = action_fixed_to_ticks(action_fixed[1]);
#else
  for(i = 0; i < OBS_SIZE; i++)
  {
    obs[i] = (float)test_case->obs_q1000[i] / 1000.0f;
  }

  if(LINE_FOLLOW_FAKE_RESET_EACH_ROW)
  {
    reset_state();
  }
  forward_model(obs, actions);
  left_ticks = action_to_ticks(actions[0]);
  right_ticks = action_to_ticks(actions[1]);
#endif

  print("M %s ", test_case->name);
  print_q1000(test_case->obs_q1000[0]);
  print(" ");
  print_q1000(test_case->obs_q1000[1]);
  print(" ");
  print_q1000(test_case->obs_q1000[2]);
  print(" ");
  print_float3(actions[0]);
  print(" ");
  print_float3(actions[1]);
  print(" %d %d\n", left_ticks, right_ticks);
}

int main(void)
{
  int i;
#if LINE_FOLLOW_FAKE_REPLAY
#include "line_follow_replay_cases.h"
#else
  FakeCase cases[] = {
    {"all_white",    {0,    0,    0}},
    {"all_black",    {1000, 1000, 1000}},
    {"left",         {1000, 0,    0}},
    {"middle",       {0,    1000, 0}},
    {"right",        {0,    0,    1000}},
    {"soft_left",    {800,  200,  0}},
    {"soft_right",   {0,    200,  800}},
    {"balanced_edge", {200, 1000, 200}},
  };
#endif
  int num_cases = sizeof(cases) / sizeof(cases[0]);

  low(26);
  low(27);
  if((LINE_FOLLOW_MODEL_RAW_FLOATS != expected_raw_float_count()
        && LINE_FOLLOW_MODEL_RAW_FLOATS != expected_aligned_read_float_count())
      || LINE_FOLLOW_MODEL_OBS_SIZE != OBS_SIZE
      || LINE_FOLLOW_MODEL_HIDDEN_SIZE != HIDDEN_SIZE
      || LINE_FOLLOW_MODEL_NUM_LAYERS != NUM_LAYERS
      || LINE_FOLLOW_MODEL_NUM_ACTIONS != NUM_ACTIONS)
  {
    print("\nLINE_FOLLOW_MODEL_FAKE shape mismatch\n");
    print("raw_floats=%d expected=%d aligned=%d hidden_size=%d num_layers=%d\n",
        LINE_FOLLOW_MODEL_RAW_FLOATS, expected_raw_float_count(),
        expected_aligned_read_float_count(), HIDDEN_SIZE, NUM_LAYERS);
    print("header obs=%d hidden=%d layers=%d actions=%d\n",
        LINE_FOLLOW_MODEL_OBS_SIZE, LINE_FOLLOW_MODEL_HIDDEN_SIZE,
        LINE_FOLLOW_MODEL_NUM_LAYERS, LINE_FOLLOW_MODEL_NUM_ACTIONS);
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
  print("hidden_size=%d num_layers=%d inference_mode=%d folded_supported=%d fixed_scale=%d\n",
      HIDDEN_SIZE, NUM_LAYERS, LINE_FOLLOW_INFERENCE_MODE,
      LINE_FOLLOW_MODEL_FOLDED_SUPPORTED, LINE_FOLLOW_MODEL_FIXED_SCALE);
  print("scale policy wheels floor to %d ticks/s, action=1 -> %d mm/s, tire=65 mm, ticks/rev=64\n",
      (int)(MIN_DRIVE_TICKS_PER_SEC), (int)(MAX_WHEEL_SPEED_MPS * 1000.0f + 0.5f));
  print("no QTI reads, no drive output, MinGRU reset_each_row=%d\n",
      LINE_FOLLOW_FAKE_RESET_EACH_ROW);
  print("M name left middle right action0 action1 left_ticks right_ticks\n");

  reset_state();
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
