/*
  Line-follow trained-model live QTI telemetry.

  This runs on the Propeller. It embeds the trained PufferLib native checkpoint,
  reads three QTI sensors, normalizes them to the sim observation polarity, runs
  the model forward pass, and prints a calibration-heavy telemetry row. Motor
  output is compiled in only when LINE_FOLLOW_ENABLE_DRIVE is set to 1.
*/

#include "simpletools.h"
#include <math.h>

#include "line_follow_model_weights.h"

#ifndef LINE_FOLLOW_ENABLE_DRIVE
#define LINE_FOLLOW_ENABLE_DRIVE 0
#endif

#if LINE_FOLLOW_ENABLE_DRIVE
#include "abdrive.h"
#endif

#ifndef QTI_LEFT_PIN
#define QTI_LEFT_PIN 7
#endif

#ifndef QTI_MIDDLE_PIN
#define QTI_MIDDLE_PIN 6
#endif

#ifndef QTI_RIGHT_PIN
#define QTI_RIGHT_PIN 5
#endif

#ifndef QTI_WHITE_TIME
#define QTI_WHITE_TIME 40
#endif

#ifndef QTI_BLACK_TIME
#define QTI_BLACK_TIME 350
#endif

#ifndef QTI_THRESHOLD_Q1000
#define QTI_THRESHOLD_Q1000 500
#endif

#ifndef LINE_FOLLOW_LOOP_MS
#define LINE_FOLLOW_LOOP_MS 0
#endif

#ifndef LINE_FOLLOW_MAX_LOOPS
#define LINE_FOLLOW_MAX_LOOPS 0
#endif

#ifndef LINE_FOLLOW_PRINT_EVERY
#define LINE_FOLLOW_PRINT_EVERY 1
#endif

#define SENSOR_COUNT 3
#define OBS_SIZE 3
#define LEFT_SENSOR 0
#define MIDDLE_SENSOR 1
#define RIGHT_SENSOR 2
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
#define COMMAND_DEADBAND 0.04f
#define TIRE_DIAMETER_M 0.065f
#define TICKS_PER_REV 64.0f
#define LINE_FOLLOW_PI 3.14159265358979323846f

static int qti_pins[SENSOR_COUNT] = {
  QTI_LEFT_PIN,
  QTI_MIDDLE_PIN,
  QTI_RIGHT_PIN
};

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
  weight_idx = (weight_idx + 3) & ~3;
  return out;
}

static void init_model(void)
{
  int i;
  weight_idx = 0;
  encoder_w = take_aligned_weights(HIDDEN_SIZE * OBS_SIZE);
  decoder_w = take_aligned_weights(DECODER_SIZE * HIDDEN_SIZE);
  (void)take_aligned_weights(NUM_ACTIONS);
#if NUM_LAYERS > 0
  mingru_w = take_aligned_weights(3 * HIDDEN_SIZE * HIDDEN_SIZE);
#endif

  for(i = 0; i < HIDDEN_SIZE; i++)
  {
    state[i] = 0.0f;
  }
}

static int clamp_int(int value, int lo, int hi)
{
  if(value < lo) return lo;
  if(value > hi) return hi;
  return value;
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

static int normalize_q1000(int raw)
{
  int denom = QTI_BLACK_TIME - QTI_WHITE_TIME;
  int obs;

  if(denom == 0)
  {
    return raw >= QTI_BLACK_TIME ? 1000 : 0;
  }

  obs = ((raw - QTI_WHITE_TIME) * 1000) / denom;
  return clamp_int(obs, 0, 1000);
}

static int read_qti_pin(int pin)
{
  high(pin);
  pause(1);
  return rc_time(pin, 1);
}

static void read_qti(int raw[SENSOR_COUNT], int obs_q1000[SENSOR_COUNT],
    float sensor_obs[SENSOR_COUNT], float model_obs[OBS_SIZE])
{
  int i;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    raw[i] = read_qti_pin(qti_pins[i]);
    obs_q1000[i] = normalize_q1000(raw[i]);
    sensor_obs[i] = (float)obs_q1000[i] / 1000.0f;
  }
  for(i = 0; i < OBS_SIZE; i++)
  {
    model_obs[i] = sensor_obs[i];
  }
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
  actions[0] = clampf_model(decoder_out[0], -1.0f, 1.0f);
  actions[1] = clampf_model(decoder_out[1], -1.0f, 1.0f);
}

static int action_to_ticks(float action)
{
  float clipped = clampf_model(action, -1.0f, 1.0f);
  float unit = clipped;
  float mps;
  float ticks;

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
  if(ticks >= 0.0f)
  {
    return (int)(ticks + 0.5f);
  }
  return (int)(ticks - 0.5f);
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

static void print_int_field(int value)
{
  print(" %d", value);
}

static unsigned int read_clock_ticks(void)
{
  volatile unsigned int ticks = CNT;
  return ticks;
}

static int elapsed_ms(unsigned int start_ticks, unsigned int end_ticks)
{
  unsigned int ticks_per_ms = (unsigned int)(CLKFREQ / 1000);
  unsigned int elapsed_ticks = end_ticks - start_ticks;
  return (int)((elapsed_ticks + ticks_per_ms / 2) / ticks_per_ms);
}

int main(void)
{
  int raw[SENSOR_COUNT];
  int obs_q1000[SENSOR_COUNT];
  int raw_min[SENSOR_COUNT] = {2147483647, 2147483647, 2147483647};
  int raw_max[SENSOR_COUNT] = {0, 0, 0};
  float sensor_obs[SENSOR_COUNT];
  float model_obs[OBS_SIZE];
  float actions[NUM_ACTIONS];
  int left_ticks = 0;
  int right_ticks = 0;
  int loops = 0;
  unsigned int loop_start_ticks = 0;
  unsigned int loop_end_ticks = 0;
  int body_ms = 0;
  int i;

  low(26);
  low(27);
  if(LINE_FOLLOW_MODEL_RAW_FLOATS != expected_raw_float_count())
  {
    print("\nLINE_FOLLOW_MODEL_LIVE shape mismatch\n");
    print("raw_floats=%d expected=%d hidden_size=%d num_layers=%d\n",
        LINE_FOLLOW_MODEL_RAW_FLOATS, expected_raw_float_count(), HIDDEN_SIZE, NUM_LAYERS);
    while(1)
    {
      pause(1000);
    }
  }
  init_model();

  print("\nLINE_FOLLOW_MODEL_LIVE v1 drive=%d\n", LINE_FOLLOW_ENABLE_DRIVE);
  print("checkpoint=%s\n", line_follow_model_checkpoint);
  print("raw_floats=%d padded_floats=%d\n",
      LINE_FOLLOW_MODEL_RAW_FLOATS, LINE_FOLLOW_MODEL_PADDED_FLOATS);
  print("hidden_size=%d num_layers=%d\n", HIDDEN_SIZE, NUM_LAYERS);
  print("pins left=%d middle=%d right=%d\n",
      QTI_LEFT_PIN, QTI_MIDDLE_PIN, QTI_RIGHT_PIN);
  print("cal white=%d black=%d threshold_q1000=%d loop_ms=%d max_loops=%d print_every=%d max_speed_mm_s=%d\n",
      QTI_WHITE_TIME, QTI_BLACK_TIME, QTI_THRESHOLD_Q1000,
      LINE_FOLLOW_LOOP_MS, LINE_FOLLOW_MAX_LOOPS, LINE_FOLLOW_PRINT_EVERY,
      (int)(MAX_WHEEL_SPEED_MPS * 1000.0f + 0.5f));
  print("clock clkfreq=%d ms_ticks=%d\n", CLKFREQ, ms);
  print("NOTE lower raw should be brighter/whiter, higher raw should be darker/blacker\n");
  print("model observations are left, middle, right; pin 4 is ignored\n");
  print("L step raw0 raw1 raw2 min0 min1 min2 max0 max1 max2 sensor0 sensor1 sensor2 model0 model1 model2 action0 action1 left right dt_ms\n");

#if LINE_FOLLOW_ENABLE_DRIVE
  drive_setRampStep(4);
  drive_speed(0, 0);
  pause(500);
#endif

  while(LINE_FOLLOW_MAX_LOOPS == 0 || loops < LINE_FOLLOW_MAX_LOOPS)
  {
    loop_start_ticks = read_clock_ticks();
    read_qti(raw, obs_q1000, sensor_obs, model_obs);
    for(i = 0; i < SENSOR_COUNT; i++)
    {
      if(raw[i] < raw_min[i]) raw_min[i] = raw[i];
      if(raw[i] > raw_max[i]) raw_max[i] = raw[i];
    }

    forward_model(model_obs, actions);
    left_ticks = action_to_ticks(actions[0]);
    right_ticks = action_to_ticks(actions[1]);
    loop_end_ticks = read_clock_ticks();
    body_ms = elapsed_ms(loop_start_ticks, loop_end_ticks);

    if(LINE_FOLLOW_PRINT_EVERY <= 1 || (loops % LINE_FOLLOW_PRINT_EVERY) == 0)
    {
      print("L %d", loops);
      for(i = 0; i < SENSOR_COUNT; i++) print_int_field(raw[i]);
      for(i = 0; i < SENSOR_COUNT; i++) print_int_field(raw_min[i]);
      for(i = 0; i < SENSOR_COUNT; i++) print_int_field(raw_max[i]);
      for(i = 0; i < SENSOR_COUNT; i++) print_int_field(obs_q1000[i]);
      for(i = 0; i < OBS_SIZE; i++) print_int_field(obs_q1000[i]);
      print(" ");
      print_float3(actions[0]);
      print(" ");
      print_float3(actions[1]);
      print(" %d %d %d\n", left_ticks, right_ticks, body_ms);
    }

#if LINE_FOLLOW_ENABLE_DRIVE
    drive_speed(left_ticks, right_ticks);
#endif

    set_output(26, obs_q1000[LEFT_SENSOR] >= QTI_THRESHOLD_Q1000);
    set_output(27, obs_q1000[RIGHT_SENSOR] >= QTI_THRESHOLD_Q1000);

    loops++;
    if(LINE_FOLLOW_LOOP_MS > 0)
    {
      pause(LINE_FOLLOW_LOOP_MS);
    }
  }

#if LINE_FOLLOW_ENABLE_DRIVE
  drive_speed(0, 0);
  pause(250);
  drive_close();
#endif

  print("LINE_FOLLOW_MODEL_LIVE done\n");
  while(1)
  {
    pause(1000);
  }
}
