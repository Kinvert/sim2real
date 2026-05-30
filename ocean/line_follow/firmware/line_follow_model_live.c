/*
  Line-follow trained-model live QTI telemetry.

  This runs on the Propeller. It embeds the trained PufferLib native checkpoint,
  reads three QTI sensors, normalizes them to the sim observation polarity, runs
  the model forward pass, and prints a calibration-heavy telemetry row. Motor
  output is compiled in only when LINE_FOLLOW_ENABLE_DRIVE is set to 1.
*/

#include "simpletools.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

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

#ifndef LINE_FOLLOW_SD_LOG
#define LINE_FOLLOW_SD_LOG 0
#endif

#ifndef LINE_FOLLOW_SD_FILE
#define LINE_FOLLOW_SD_FILE "lf_log.bin"
#endif

#ifndef LINE_FOLLOW_SD_AUTO_NAME
#define LINE_FOLLOW_SD_AUTO_NAME 0
#endif

#ifndef LINE_FOLLOW_SD_MAX_RECORDS
#define LINE_FOLLOW_SD_MAX_RECORDS 6000
#endif

#ifndef LINE_FOLLOW_SD_LOG_EVERY
#define LINE_FOLLOW_SD_LOG_EVERY 1
#endif

#ifndef LINE_FOLLOW_SD_FLUSH_EVERY
#define LINE_FOLLOW_SD_FLUSH_EVERY 64
#endif

#ifndef LINE_FOLLOW_SD_DO_PIN
#define LINE_FOLLOW_SD_DO_PIN 22
#endif

#ifndef LINE_FOLLOW_SD_CLK_PIN
#define LINE_FOLLOW_SD_CLK_PIN 23
#endif

#ifndef LINE_FOLLOW_SD_DI_PIN
#define LINE_FOLLOW_SD_DI_PIN 24
#endif

#ifndef LINE_FOLLOW_SD_CS_PIN
#define LINE_FOLLOW_SD_CS_PIN 25
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
#if NUM_LAYERS == 0 && LINE_FOLLOW_MODEL_FOLDED_SUPPORTED
#define LINE_FOLLOW_INFERENCE_MODE LINE_FOLLOW_INFERENCE_FIXED
#else
#define LINE_FOLLOW_INFERENCE_MODE LINE_FOLLOW_INFERENCE_FULL
#endif
#endif

#if LINE_FOLLOW_INFERENCE_MODE < LINE_FOLLOW_INFERENCE_FULL || LINE_FOLLOW_INFERENCE_MODE > LINE_FOLLOW_INFERENCE_FIXED
#error "LINE_FOLLOW_INFERENCE_MODE must be 0=full, 1=folded-float, or 2=fixed"
#endif
#if LINE_FOLLOW_INFERENCE_MODE != LINE_FOLLOW_INFERENCE_FULL && !(NUM_LAYERS == 0 && LINE_FOLLOW_MODEL_FOLDED_SUPPORTED)
#error "folded inference requires NUM_LAYERS=0 and a folded model header"
#endif

#define SD_LOG_MAGIC 0x4453464cu
#define SD_LOG_LAST_FILE "lf_last.txt"

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

#if LINE_FOLLOW_SD_LOG
static char sd_log_file[16];

typedef struct SdLogHeader {
  uint32_t magic;
  int32_t version;
  int32_t header_size;
  int32_t row_size;
  int32_t sensor_count;
  int32_t obs_size;
  int32_t hidden_size;
  int32_t num_layers;
  int32_t clkfreq;
  int32_t qti_white_time;
  int32_t qti_black_time;
  int32_t qti_threshold_q1000;
  int32_t max_speed_mm_s;
  int32_t min_drive_ticks_per_sec;
  int32_t command_deadband_q1000;
  int32_t loop_pause_ms;
  int32_t sd_log_every;
  int32_t sd_max_records;
  int32_t model_raw_float_count;
  char checkpoint[128];
} SdLogHeader;

typedef struct SdLogRow {
  int32_t loop;
  int32_t elapsed_ms;
  int32_t period_ms;
  int32_t body_ms;
  int32_t raw[SENSOR_COUNT];
  int32_t encoder_left_ticks;
  int32_t encoder_right_ticks;
  int32_t obs_q1000[SENSOR_COUNT];
  int32_t action_q1000[NUM_ACTIONS];
  int32_t command_left_ticks;
  int32_t command_right_ticks;
  int32_t flags;
} SdLogRow;
#endif

static int align4_int(int value)
{
  return (value + 3) & ~3;
}

static int expected_raw_float_count(void)
{
  int index = 0;
  int layer;
  index += HIDDEN_SIZE * OBS_SIZE;
  index = align4_int(index);
  index += DECODER_SIZE * HIDDEN_SIZE;
  index = align4_int(index);
  index += NUM_ACTIONS;
#if NUM_LAYERS > 0
  index = align4_int(index);
  for(layer = 0; layer < NUM_LAYERS; layer++)
  {
    index += 3 * HIDDEN_SIZE * HIDDEN_SIZE;
    index = align4_int(index);
  }
#else
  (void)layer;
#endif
  return index;
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
#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FULL
  encoder_w = take_aligned_weights(HIDDEN_SIZE * OBS_SIZE);
  decoder_w = take_aligned_weights(DECODER_SIZE * HIDDEN_SIZE);
  (void)take_aligned_weights(NUM_ACTIONS);
#if NUM_LAYERS > 0
  mingru_w = take_aligned_weights(3 * HIDDEN_SIZE * HIDDEN_SIZE);
#endif
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

static int round_div_int(int numerator, int denominator)
{
  if(numerator >= 0)
  {
    return (numerator + denominator / 2) / denominator;
  }
  return -((-numerator + denominator / 2) / denominator);
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
    float model_obs[OBS_SIZE])
{
  int i;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    raw[i] = read_qti_pin(qti_pins[i]);
    obs_q1000[i] = normalize_q1000(raw[i]);
  }
#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FIXED
  (void)model_obs;
#else
  for(i = 0; i < OBS_SIZE; i++)
  {
    model_obs[i] = (float)obs_q1000[i] / 1000.0f;
  }
#endif
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
  int action;
  for(action = 0; action < NUM_ACTIONS; action++)
  {
    int i;
    int total = 0;
    for(i = 0; i < OBS_SIZE; i++)
    {
      total += obs_q1000[i]
        * line_follow_model_folded_action_w_q[action * OBS_SIZE + i];
    }
    action_fixed[action] = clamp_int(
        round_div_int(total, 1000),
        -LINE_FOLLOW_MODEL_FIXED_SCALE, LINE_FOLLOW_MODEL_FIXED_SCALE);
    actions[action] = (float)action_fixed[action]
      / (float)LINE_FOLLOW_MODEL_FIXED_SCALE;
  }
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

#if LINE_FOLLOW_SD_LOG
static int float_q1000(float value)
{
  if(value >= 0.0f)
  {
    return (int)(value * 1000.0f + 0.5f);
  }
  return (int)(value * 1000.0f - 0.5f);
}

static void copy_checkpoint(char dest[128])
{
  int i;
  for(i = 0; i < 127 && line_follow_model_checkpoint[i] != 0; i++)
  {
    dest[i] = line_follow_model_checkpoint[i];
  }
  dest[i] = 0;
  for(i = i + 1; i < 128; i++)
  {
    dest[i] = 0;
  }
}

static int copy_text(char* dest, const char* src, int max_len)
{
  int i;
  for(i = 0; i < max_len - 1 && src[i] != 0; i++)
  {
    dest[i] = src[i];
  }
  dest[i] = 0;
  return i;
}

static void make_auto_log_name(char* dest, int index)
{
  if(index < 0) index = 0;
  if(index > 999) index = 999;
  dest[0] = 'l';
  dest[1] = 'f';
  dest[2] = (char)('0' + (index / 100) % 10);
  dest[3] = (char)('0' + (index / 10) % 10);
  dest[4] = (char)('0' + index % 10);
  dest[5] = '.';
  dest[6] = 'b';
  dest[7] = 'i';
  dest[8] = 'n';
  dest[9] = 0;
}

static void write_last_log_file(const char* path)
{
  FILE* fp = fopen(SD_LOG_LAST_FILE, "w");
  int len = 0;
  if(!fp)
  {
    print("SD last-file write skipped\n");
    return;
  }
  while(path[len] != 0) len++;
  fwrite(path, 1, len, fp);
  fwrite("\n", 1, 1, fp);
  fclose(fp);
}

static FILE* open_selected_log_file(void)
{
#if LINE_FOLLOW_SD_AUTO_NAME
  int index;
  for(index = 0; index < 1000; index++)
  {
    FILE* existing;
    make_auto_log_name(sd_log_file, index);
    existing = fopen(sd_log_file, "r");
    if(existing)
    {
      fclose(existing);
      continue;
    }
    return fopen(sd_log_file, "w");
  }
  make_auto_log_name(sd_log_file, 999);
  return 0;
#else
  copy_text(sd_log_file, LINE_FOLLOW_SD_FILE, sizeof(sd_log_file));
  return fopen(sd_log_file, "w");
#endif
}

static void fill_sd_header(SdLogHeader* header)
{
  header->magic = SD_LOG_MAGIC;
  header->version = 1;
  header->header_size = (int32_t)sizeof(SdLogHeader);
  header->row_size = (int32_t)sizeof(SdLogRow);
  header->sensor_count = SENSOR_COUNT;
  header->obs_size = OBS_SIZE;
  header->hidden_size = HIDDEN_SIZE;
  header->num_layers = NUM_LAYERS;
  header->clkfreq = CLKFREQ;
  header->qti_white_time = QTI_WHITE_TIME;
  header->qti_black_time = QTI_BLACK_TIME;
  header->qti_threshold_q1000 = QTI_THRESHOLD_Q1000;
  header->max_speed_mm_s = (int32_t)(MAX_WHEEL_SPEED_MPS * 1000.0f + 0.5f);
  header->min_drive_ticks_per_sec = MIN_DRIVE_TICKS_PER_SEC;
  header->command_deadband_q1000 = (int32_t)(COMMAND_DEADBAND * 1000.0f + 0.5f);
  header->loop_pause_ms = LINE_FOLLOW_LOOP_MS;
  header->sd_log_every = LINE_FOLLOW_SD_LOG_EVERY;
  header->sd_max_records = LINE_FOLLOW_SD_MAX_RECORDS;
  header->model_raw_float_count = LINE_FOLLOW_MODEL_RAW_FLOATS;
  copy_checkpoint(header->checkpoint);
}

static FILE* open_sd_log(void)
{
  FILE* fp;
  SdLogHeader header;
  int err = sd_mount(LINE_FOLLOW_SD_DO_PIN, LINE_FOLLOW_SD_CLK_PIN,
      LINE_FOLLOW_SD_DI_PIN, LINE_FOLLOW_SD_CS_PIN);

  if(err)
  {
    print("SD mount failed err=%d\n", err);
    return 0;
  }

  fp = open_selected_log_file();
  if(!fp)
  {
    print("SD fopen failed path=%s auto=%d\n",
        sd_log_file, LINE_FOLLOW_SD_AUTO_NAME);
    return 0;
  }

  fill_sd_header(&header);
  if(fwrite(&header, sizeof(header), 1, fp) != 1)
  {
    print("SD header write failed\n");
    fclose(fp);
    return 0;
  }

  write_last_log_file(sd_log_file);
  print("SD logging to %s row_size=%d max_records=%d auto=%d\n",
      sd_log_file, (int)sizeof(SdLogRow), LINE_FOLLOW_SD_MAX_RECORDS,
      LINE_FOLLOW_SD_AUTO_NAME);
  return fp;
}

static int write_sd_row(FILE* fp, int loop, int elapsed_total_ms,
    int period_ms, int body_ms, int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT], float actions[NUM_ACTIONS],
    int left_ticks, int right_ticks, int encoder_left_ticks,
    int encoder_right_ticks)
{
  SdLogRow row;
  int i;

  row.loop = loop;
  row.elapsed_ms = elapsed_total_ms;
  row.period_ms = period_ms;
  row.body_ms = body_ms;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    row.raw[i] = raw[i];
    row.obs_q1000[i] = obs_q1000[i];
  }
  row.encoder_left_ticks = encoder_left_ticks;
  row.encoder_right_ticks = encoder_right_ticks;
  row.action_q1000[0] = float_q1000(actions[0]);
  row.action_q1000[1] = float_q1000(actions[1]);
  row.command_left_ticks = left_ticks;
  row.command_right_ticks = right_ticks;
  row.flags = 0;
  if(obs_q1000[LEFT_SENSOR] >= QTI_THRESHOLD_Q1000) row.flags |= 1;
  if(obs_q1000[MIDDLE_SENSOR] >= QTI_THRESHOLD_Q1000) row.flags |= 2;
  if(obs_q1000[RIGHT_SENSOR] >= QTI_THRESHOLD_Q1000) row.flags |= 4;

  return fwrite(&row, sizeof(row), 1, fp) == 1 ? 0 : -1;
}

static void blink_sd_error_forever(void)
{
  while(1)
  {
    high(26);
    high(27);
    pause(250);
    low(26);
    low(27);
    pause(250);
  }
}
#endif

int main(void)
{
  int raw[SENSOR_COUNT];
  int obs_q1000[SENSOR_COUNT];
  int raw_min[SENSOR_COUNT] = {2147483647, 2147483647, 2147483647};
  int raw_max[SENSOR_COUNT] = {0, 0, 0};
  float model_obs[OBS_SIZE];
  float actions[NUM_ACTIONS];
  int action_fixed[NUM_ACTIONS] = {0, 0};
  int left_ticks = 0;
  int right_ticks = 0;
  int loops = 0;
  unsigned int loop_start_ticks = 0;
  unsigned int prev_loop_start_ticks = 0;
  unsigned int run_start_ticks = 0;
  unsigned int loop_end_ticks = 0;
  int body_ms = 0;
  int period_ms = 0;
  int elapsed_total_ms = 0;
  int i;
#if LINE_FOLLOW_SD_LOG
  FILE* sd_fp = 0;
  int sd_records = 0;
  int sd_failed = 0;
  int encoder_left_ticks = 0;
  int encoder_right_ticks = 0;
#endif

  low(26);
  low(27);
  if(LINE_FOLLOW_MODEL_RAW_FLOATS != expected_raw_float_count()
      || LINE_FOLLOW_MODEL_OBS_SIZE != OBS_SIZE
      || LINE_FOLLOW_MODEL_HIDDEN_SIZE != HIDDEN_SIZE
      || LINE_FOLLOW_MODEL_NUM_LAYERS != NUM_LAYERS
      || LINE_FOLLOW_MODEL_NUM_ACTIONS != NUM_ACTIONS)
  {
    print("\nLINE_FOLLOW_MODEL_LIVE shape mismatch\n");
    print("raw_floats=%d expected=%d hidden_size=%d num_layers=%d\n",
        LINE_FOLLOW_MODEL_RAW_FLOATS, expected_raw_float_count(), HIDDEN_SIZE, NUM_LAYERS);
    print("header obs=%d hidden=%d layers=%d actions=%d\n",
        LINE_FOLLOW_MODEL_OBS_SIZE, LINE_FOLLOW_MODEL_HIDDEN_SIZE,
        LINE_FOLLOW_MODEL_NUM_LAYERS, LINE_FOLLOW_MODEL_NUM_ACTIONS);
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
  print("hidden_size=%d num_layers=%d inference_mode=%d folded_supported=%d fixed_scale=%d\n",
      HIDDEN_SIZE, NUM_LAYERS, LINE_FOLLOW_INFERENCE_MODE,
      LINE_FOLLOW_MODEL_FOLDED_SUPPORTED, LINE_FOLLOW_MODEL_FIXED_SCALE);
  print("pins left=%d middle=%d right=%d\n",
      QTI_LEFT_PIN, QTI_MIDDLE_PIN, QTI_RIGHT_PIN);
  print("cal white=%d black=%d threshold_q1000=%d loop_ms=%d max_loops=%d print_every=%d max_speed_mm_s=%d deadband_q1000=%d min_ticks=%d\n",
      QTI_WHITE_TIME, QTI_BLACK_TIME, QTI_THRESHOLD_Q1000,
      LINE_FOLLOW_LOOP_MS, LINE_FOLLOW_MAX_LOOPS, LINE_FOLLOW_PRINT_EVERY,
      (int)(MAX_WHEEL_SPEED_MPS * 1000.0f + 0.5f),
      (int)(COMMAND_DEADBAND * 1000.0f + 0.5f),
      (int)(MIN_DRIVE_TICKS_PER_SEC));
  print("sd_log=%d sd_file=%s sd_max_records=%d sd_log_every=%d sd_flush_every=%d\n",
      LINE_FOLLOW_SD_LOG, LINE_FOLLOW_SD_FILE, LINE_FOLLOW_SD_MAX_RECORDS,
      LINE_FOLLOW_SD_LOG_EVERY, LINE_FOLLOW_SD_FLUSH_EVERY);
  print("sd_auto_name=%d sd_last_file=%s\n",
      LINE_FOLLOW_SD_AUTO_NAME, SD_LOG_LAST_FILE);
  print("clock clkfreq=%d ms_ticks=%d\n", CLKFREQ, ms);
  print("NOTE lower raw should be brighter/whiter, higher raw should be darker/blacker\n");
  print("model observations are left, middle, right; pin 4 is ignored\n");
  print("L step raw0 raw1 raw2 min0 min1 min2 max0 max1 max2 sensor0 sensor1 sensor2 model0 model1 model2 action0 action1 left right dt_ms\n");

#if LINE_FOLLOW_SD_LOG
  sd_fp = open_sd_log();
  if(!sd_fp)
  {
    print("SD logging is required for this build; motors will not start.\n");
    blink_sd_error_forever();
  }
#endif

#if LINE_FOLLOW_ENABLE_DRIVE
  drive_setRampStep(4);
  drive_speed(0, 0);
  pause(500);
#endif

  while(LINE_FOLLOW_MAX_LOOPS == 0 || loops < LINE_FOLLOW_MAX_LOOPS)
  {
#if LINE_FOLLOW_SD_LOG
    if(sd_failed) break;
    if(LINE_FOLLOW_SD_MAX_RECORDS > 0 && sd_records >= LINE_FOLLOW_SD_MAX_RECORDS) break;
#endif
    loop_start_ticks = read_clock_ticks();
    if(run_start_ticks == 0)
    {
      run_start_ticks = loop_start_ticks;
    }
    elapsed_total_ms = elapsed_ms(run_start_ticks, loop_start_ticks);
    period_ms = prev_loop_start_ticks == 0
        ? 0
        : elapsed_ms(prev_loop_start_ticks, loop_start_ticks);
    prev_loop_start_ticks = loop_start_ticks;
    read_qti(raw, obs_q1000, model_obs);
    for(i = 0; i < SENSOR_COUNT; i++)
    {
      if(raw[i] < raw_min[i]) raw_min[i] = raw[i];
      if(raw[i] > raw_max[i]) raw_max[i] = raw[i];
    }

#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FIXED
    forward_model_fixed(obs_q1000, action_fixed, actions);
    left_ticks = action_fixed_to_ticks(action_fixed[0]);
    right_ticks = action_fixed_to_ticks(action_fixed[1]);
#else
    forward_model(model_obs, actions);
    left_ticks = action_to_ticks(actions[0]);
    right_ticks = action_to_ticks(actions[1]);
#endif
    loop_end_ticks = read_clock_ticks();
    body_ms = elapsed_ms(loop_start_ticks, loop_end_ticks);

    if(LINE_FOLLOW_PRINT_EVERY > 0 && (loops % LINE_FOLLOW_PRINT_EVERY) == 0)
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

#if LINE_FOLLOW_SD_LOG
#if LINE_FOLLOW_ENABLE_DRIVE
    drive_getTicks(&encoder_left_ticks, &encoder_right_ticks);
#endif
    if(LINE_FOLLOW_SD_LOG_EVERY > 0 && (loops % LINE_FOLLOW_SD_LOG_EVERY) == 0)
    {
      if(write_sd_row(sd_fp, loops, elapsed_total_ms, period_ms, body_ms,
          raw, obs_q1000, actions, left_ticks, right_ticks,
          encoder_left_ticks, encoder_right_ticks))
      {
        print("SD row write failed at loop=%d\n", loops);
        sd_failed = 1;
      }
      else
      {
        sd_records++;
        if(LINE_FOLLOW_SD_FLUSH_EVERY > 0
            && (sd_records % LINE_FOLLOW_SD_FLUSH_EVERY) == 0)
        {
          fflush(sd_fp);
        }
      }
    }
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

#if LINE_FOLLOW_SD_LOG
  if(sd_fp)
  {
    fflush(sd_fp);
    fclose(sd_fp);
    print("SD log closed records=%d\n", sd_records);
  }
#endif

  print("LINE_FOLLOW_MODEL_LIVE done\n");
  while(1)
  {
    pause(1000);
  }
}
