/*
  Line-follow trained-model live QTI telemetry.

  This runs on the Propeller. It embeds the trained PufferLib native checkpoint,
  reads three QTI sensors, normalizes them to the sim observation polarity, runs
  the model forward pass, and prints a calibration-heavy telemetry row. Motor
  output is compiled in only when LINE_FOLLOW_ENABLE_DRIVE is set to 1.
*/

#include "simpletools.h"
#include <propeller.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "line_follow_qti_sampler.h"
#include "line_follow_qti_sampler4.h"
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

#ifndef QTI_FAR_RIGHT_PIN
#define QTI_FAR_RIGHT_PIN 4
#endif

#define LINE_FOLLOW_QTI_MAP_THREE_PIN 0
#define LINE_FOLLOW_QTI_MAP_FOUR_ADJACENT_MAX 1
#define LINE_FOLLOW_QTI_MAP_FOUR_CENTER_MAX 2
#ifndef LINE_FOLLOW_QTI_MAP_MODE
#define LINE_FOLLOW_QTI_MAP_MODE LINE_FOLLOW_QTI_MAP_THREE_PIN
#endif

#if LINE_FOLLOW_QTI_MAP_MODE < LINE_FOLLOW_QTI_MAP_THREE_PIN || LINE_FOLLOW_QTI_MAP_MODE > LINE_FOLLOW_QTI_MAP_FOUR_CENTER_MAX
#error "LINE_FOLLOW_QTI_MAP_MODE must be 0=three-pin, 1=four-adjacent-max, or 2=four-center-max"
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

#ifndef LINE_FOLLOW_OBS_SUBTRACT_MIN
#define LINE_FOLLOW_OBS_SUBTRACT_MIN 0
#endif

#ifndef LINE_FOLLOW_OBS_CONTRAST_GAIN_Q1000
#define LINE_FOLLOW_OBS_CONTRAST_GAIN_Q1000 1000
#endif

#ifndef LINE_FOLLOW_OBS_RAW_DIFF
#define LINE_FOLLOW_OBS_RAW_DIFF 0
#endif

#ifndef LINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000
#define LINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000 1000
#endif

#ifndef LINE_FOLLOW_OBS_RAW_DIFF_SPAN
#define LINE_FOLLOW_OBS_RAW_DIFF_SPAN 0
#endif

#ifndef QTI_CHARGE_US
#define QTI_CHARGE_US 1000
#endif

#ifndef QTI_TIMEOUT_US
#define QTI_TIMEOUT_US 1000
#endif

#ifndef QTI_SAMPLE_PERIOD_US
#define QTI_SAMPLE_PERIOD_US 0
#endif

#define LINE_FOLLOW_QTI_MODE_SEQUENTIAL 0
#define LINE_FOLLOW_QTI_MODE_GROUPED_C 1
#define LINE_FOLLOW_QTI_MODE_PASM_COG 2
#ifndef LINE_FOLLOW_QTI_MODE
#define LINE_FOLLOW_QTI_MODE LINE_FOLLOW_QTI_MODE_PASM_COG
#endif

#if LINE_FOLLOW_QTI_MODE < LINE_FOLLOW_QTI_MODE_SEQUENTIAL || LINE_FOLLOW_QTI_MODE > LINE_FOLLOW_QTI_MODE_PASM_COG
#error "LINE_FOLLOW_QTI_MODE must be 0=sequential, 1=grouped C, or 2=PASM cog"
#endif

#ifndef LINE_FOLLOW_LOOP_MS
#define LINE_FOLLOW_LOOP_MS 0
#endif

#ifndef LINE_FOLLOW_MAX_LOOPS
#define LINE_FOLLOW_MAX_LOOPS 0
#endif

#ifndef LINE_FOLLOW_RUN_MS
#define LINE_FOLLOW_RUN_MS 0
#endif

#ifndef LINE_FOLLOW_PRINT_EVERY
#define LINE_FOLLOW_PRINT_EVERY 1
#endif

#ifndef LINE_FOLLOW_START_DELAY_MS
#define LINE_FOLLOW_START_DELAY_MS 0
#endif

#ifndef LINE_FOLLOW_STARTUP_PRINTS
#define LINE_FOLLOW_STARTUP_PRINTS 1
#endif

#ifndef LINE_FOLLOW_STATUS_LEDS
#define LINE_FOLLOW_STATUS_LEDS 0
#endif

#ifndef LINE_FOLLOW_SD_LOG
#define LINE_FOLLOW_SD_LOG 0
#endif

#ifndef LINE_FOLLOW_SD_REQUIRED
#define LINE_FOLLOW_SD_REQUIRED 1
#endif

#ifndef LINE_FOLLOW_SD_STOP_ON_WRITE_FAIL
#define LINE_FOLLOW_SD_STOP_ON_WRITE_FAIL LINE_FOLLOW_SD_REQUIRED
#endif

#ifndef LINE_FOLLOW_SD_FILE
#define LINE_FOLLOW_SD_FILE "lf_log.bin"
#endif

#ifndef LINE_FOLLOW_SD_PREV_FILE
#define LINE_FOLLOW_SD_PREV_FILE "lfprev.bin"
#endif

#ifndef LINE_FOLLOW_SD_BACKUP_PREVIOUS
#define LINE_FOLLOW_SD_BACKUP_PREVIOUS 0
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

#ifndef LINE_FOLLOW_SD_CHUNK_RECORDS
#define LINE_FOLLOW_SD_CHUNK_RECORDS 0
#endif

#ifndef LINE_FOLLOW_SD_DEFERRED
#define LINE_FOLLOW_SD_DEFERRED 0
#endif

#ifndef LINE_FOLLOW_SD_COG_LOG
#define LINE_FOLLOW_SD_COG_LOG 0
#endif

#ifndef LINE_FOLLOW_TRACE_SLOTS
#define LINE_FOLLOW_TRACE_SLOTS 64
#endif

#ifndef LINE_FOLLOW_TRACE_PERIOD
#define LINE_FOLLOW_TRACE_PERIOD 16
#endif

#ifndef LINE_FOLLOW_DEFERRED_EVENT_SLOTS
#define LINE_FOLLOW_DEFERRED_EVENT_SLOTS 12
#endif

#ifndef LINE_FOLLOW_SD_COG_STACK_WORDS
#define LINE_FOLLOW_SD_COG_STACK_WORDS 160
#endif

#ifndef LINE_FOLLOW_SD_WRITE_TIMEOUT_MS
#define LINE_FOLLOW_SD_WRITE_TIMEOUT_MS 15000
#endif

#ifndef LINE_FOLLOW_BUILD_ID
#define LINE_FOLLOW_BUILD_ID 0
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
#define LINE_FOLLOW_INFERENCE_MODE LINE_FOLLOW_INFERENCE_FIXED
#endif

#if LINE_FOLLOW_INFERENCE_MODE < LINE_FOLLOW_INFERENCE_FULL || LINE_FOLLOW_INFERENCE_MODE > LINE_FOLLOW_INFERENCE_FIXED
#error "LINE_FOLLOW_INFERENCE_MODE must be 0=full, 1=folded-float, or 2=fixed"
#endif
#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FOLDED_FLOAT && !(NUM_LAYERS == 0 && LINE_FOLLOW_MODEL_FOLDED_SUPPORTED)
#error "folded inference requires NUM_LAYERS=0 and a folded model header"
#endif

#define SD_LOG_MAGIC 0x4453464cu
#define SD_LOG_LAST_FILE "lf_last.txt"

static int qti_pins[SENSOR_COUNT] = {
  QTI_LEFT_PIN,
  QTI_MIDDLE_PIN,
  QTI_RIGHT_PIN
};

static unsigned int qti_pin_masks[SENSOR_COUNT] = {
  1u << QTI_LEFT_PIN,
  1u << QTI_MIDDLE_PIN,
  1u << QTI_RIGHT_PIN
};
static unsigned int qti_pin_mask =
  (1u << QTI_LEFT_PIN) | (1u << QTI_MIDDLE_PIN) | (1u << QTI_RIGHT_PIN);

#if LINE_FOLLOW_QTI_MAP_MODE != LINE_FOLLOW_QTI_MAP_THREE_PIN
#define QTI_PHYSICAL_COUNT 4
static int qti4_pins[QTI_PHYSICAL_COUNT] = {
  QTI_LEFT_PIN,
  QTI_MIDDLE_PIN,
  QTI_RIGHT_PIN,
  QTI_FAR_RIGHT_PIN
};
static unsigned int qti4_pin_masks[QTI_PHYSICAL_COUNT] = {
  1u << QTI_LEFT_PIN,
  1u << QTI_MIDDLE_PIN,
  1u << QTI_RIGHT_PIN,
  1u << QTI_FAR_RIGHT_PIN
};
static unsigned int qti4_pin_mask =
  (1u << QTI_LEFT_PIN) | (1u << QTI_MIDDLE_PIN)
  | (1u << QTI_RIGHT_PIN) | (1u << QTI_FAR_RIGHT_PIN);
#endif

static int qti_wait_us = 0;
static int qti_sample_us = 0;
static int qti_sample_period_us = 0;
static unsigned int qti_last_seq = 0;
static int qti_last_physical_count = 0;
static int qti_last_physical_raw[4] = {0, 0, 0, 0};

static void store_qti_physical_raw(const int* raw, int count)
{
  int i;
  if(count > 4)
  {
    count = 4;
  }
  qti_last_physical_count = count;
  for(i = 0; i < count; i++)
  {
    qti_last_physical_raw[i] = raw[i];
  }
  for(; i < 4; i++)
  {
    qti_last_physical_raw[i] = 0;
  }
}

#if LINE_FOLLOW_QTI_MODE == LINE_FOLLOW_QTI_MODE_PASM_COG && LINE_FOLLOW_QTI_MAP_MODE == LINE_FOLLOW_QTI_MAP_THREE_PIN
extern unsigned int _load_start_coguser0[];
static volatile LineFollowQtiSamplerMailbox qti_mailbox;
static int qti_cog_id = -1;
#elif LINE_FOLLOW_QTI_MODE == LINE_FOLLOW_QTI_MODE_PASM_COG && LINE_FOLLOW_QTI_MAP_MODE != LINE_FOLLOW_QTI_MAP_THREE_PIN
extern unsigned int _load_start_coguser0[];
static volatile LineFollowQtiSampler4Mailbox qti4_mailbox;
static int qti_cog_id = -1;
#endif

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
  int32_t qti_mode;
  int32_t qti_charge_us;
  int32_t qti_timeout_us;
  int32_t qti_sample_period_us;
  int32_t start_delay_ms;
  int32_t run_ms;
  int32_t inference_mode;
  int32_t build_id;
} SdLogHeader;

typedef struct SdLogRow {
  int32_t loop;
  int32_t elapsed_ms;
  int32_t period_ms;
  int32_t body_ms;
  int32_t period_us;
  int32_t body_us;
  int32_t qti_wait_us;
  int32_t policy_us;
  int32_t drive_us;
  int32_t qti_sample_us;
  int32_t raw[SENSOR_COUNT];
  int32_t encoder_left_ticks;
  int32_t encoder_right_ticks;
  int32_t obs_q1000[SENSOR_COUNT];
  int32_t action_q1000[NUM_ACTIONS];
  int32_t command_left_ticks;
  int32_t command_right_ticks;
  int32_t flags;
} SdLogRow;

#if LINE_FOLLOW_SD_COG_LOG
typedef struct SdCogMailbox {
  volatile int32_t control;
  volatile uint32_t seq;
  volatile int32_t status;
  volatile int32_t records;
  volatile int32_t dropped;
  SdLogRow row;
} SdCogMailbox;

#define SD_COG_IDLE 0
#define SD_COG_OPENING 1
#define SD_COG_WRITING 2
#define SD_COG_DONE 3
#define SD_COG_ERROR -1

static volatile SdCogMailbox sd_cog_mailbox;
static int sd_cog_id = -1;
static int sd_cog_stack[LINE_FOLLOW_SD_COG_STACK_WORDS];

typedef struct CompactTraceRow {
  int32_t loop;
  int32_t elapsed_ms;
  int32_t period_us;
  int16_t raw[SENSOR_COUNT];
  int16_t obs_q1000[SENSOR_COUNT];
  int16_t action_q1000[NUM_ACTIONS];
  int16_t command_ticks[NUM_ACTIONS];
  int16_t flags;
  int16_t kind;
} CompactTraceRow;

static CompactTraceRow trace_rows[LINE_FOLLOW_TRACE_SLOTS];
static int trace_count = 0;
static int trace_seen = 0;
static int trace_dropped = 0;
static int trace_last_left = -32768;
static int trace_last_right = -32768;
static int trace_raw_min[SENSOR_COUNT];
static int trace_raw_max[SENSOR_COUNT];
static int trace_obs_min[SENSOR_COUNT];
static int trace_obs_max[SENSOR_COUNT];
static int trace_command_min[NUM_ACTIONS];
static int trace_command_max[NUM_ACTIONS];
static int trace_or_flags = 0;
static int trace_all_white_count = 0;
static int trace_any_black_count = 0;
#endif

#if LINE_FOLLOW_SD_DEFERRED
#define DEFER_EVENT_FIRST 1
#define DEFER_EVENT_LAST 2
#define DEFER_EVENT_STRONGEST 3
#define DEFER_EVENT_SLOWEST_LINE 4
#define DEFER_EVENT_BOTH_FLOOR 10
#define DEFER_EVENT_PAIR_EDGE 11
#define DEFER_EVENT_COMMAND_CHANGE 12
#define DEFER_EVENT_PERIODIC 13

#define LOGGER_IDLE 0
#define LOGGER_REQUESTED 1
#define LOGGER_WRITING 2
#define LOGGER_DONE 3
#define LOGGER_ERROR -1

typedef struct DeferredSnapshot {
  int32_t kind;
  int32_t loop;
  int32_t elapsed_ms;
  int32_t period_us;
  int16_t raw[SENSOR_COUNT];
  int16_t obs_q1000[SENSOR_COUNT];
  int16_t action_q1000[NUM_ACTIONS];
  int16_t command_ticks[NUM_ACTIONS];
  int16_t flags;
} DeferredSnapshot;

typedef struct DeferredLog {
  int32_t loops;
  int32_t final_elapsed_ms;
  int32_t or_flags;
  int32_t raw_min[SENSOR_COUNT];
  int32_t raw_max[SENSOR_COUNT];
  int32_t raw_sum[SENSOR_COUNT];
  int32_t obs_min[SENSOR_COUNT];
  int32_t obs_max[SENSOR_COUNT];
  int32_t obs_sum[SENSOR_COUNT];
  int32_t action_min[NUM_ACTIONS];
  int32_t action_max[NUM_ACTIONS];
  int32_t action_sum[NUM_ACTIONS];
  int32_t command_min[NUM_ACTIONS];
  int32_t command_max[NUM_ACTIONS];
  int32_t command_sum[NUM_ACTIONS];
  int32_t period_us_min;
  int32_t period_us_max;
  int32_t period_us_sum;
  int32_t body_us_min;
  int32_t body_us_max;
  int32_t body_us_sum;
  int32_t qti_us_min;
  int32_t qti_us_max;
  int32_t qti_us_sum;
  int32_t policy_us_min;
  int32_t policy_us_max;
  int32_t policy_us_sum;
  int32_t drive_us_min;
  int32_t drive_us_max;
  int32_t drive_us_sum;
  int32_t sample_us_min;
  int32_t sample_us_max;
  int32_t sample_us_sum;
  int32_t all_white_count;
  int32_t any_black_count;
  int32_t black_count[SENSOR_COUNT];
  int32_t pair_edge_count;
  int32_t one_floor_count;
  int32_t both_floor_count;
  int32_t command_change_count;
  int32_t prev_command[NUM_ACTIONS];
  int32_t have_prev_command;
  int32_t strongest_obs;
  int32_t slowest_line_command;
  DeferredSnapshot first;
  DeferredSnapshot last;
  DeferredSnapshot strongest;
  DeferredSnapshot slowest_line;
  DeferredSnapshot events[LINE_FOLLOW_DEFERRED_EVENT_SLOTS];
  int32_t event_count;
} DeferredLog;

static DeferredLog deferred_log;
static volatile int sd_logger_state = LOGGER_IDLE;
static volatile int sd_logger_records = 0;
static int sd_logger_cog = -1;
static int sd_logger_stack[LINE_FOLLOW_SD_COG_STACK_WORDS];
#endif
#endif

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
  int i;
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

  for(i = 0; i < HIDDEN_SIZE; i++)
  {
    state[i] = 0.0f;
    state_q[i] = 0;
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

static int round_div_ll(long long numerator, int denominator)
{
  if(numerator >= 0)
  {
    return (int)((numerator + denominator / 2) / denominator);
  }
  return (int)(-((-numerator + denominator / 2) / denominator));
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

static void postprocess_obs_q1000(const int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT])
{
#if LINE_FOLLOW_OBS_RAW_DIFF
  int i;
  int baseline = raw[0];
  int span = LINE_FOLLOW_OBS_RAW_DIFF_SPAN;
  if(span <= 0)
  {
    span = QTI_BLACK_TIME - QTI_WHITE_TIME;
  }
  if(span <= 0)
  {
    span = 1;
  }
  for(i = 1; i < SENSOR_COUNT; i++)
  {
    if(raw[i] < baseline)
    {
      baseline = raw[i];
    }
  }
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    int value = raw[i] - baseline;
    int gain = LINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000;
    value = round_div_int(value * 1000, span);
    if(gain <= 0 || value <= 0)
    {
      value = 0;
    }
    else
    {
      int saturating_value = (1000 * 1000 + gain - 1) / gain;
      if(value >= saturating_value)
      {
        value = 1000;
      }
      else
      {
        value = round_div_int(value * gain, 1000);
      }
    }
    obs_q1000[i] = clamp_int(value, 0, 1000);
  }
#elif LINE_FOLLOW_OBS_SUBTRACT_MIN
  int i;
  int baseline = obs_q1000[0];
  for(i = 1; i < SENSOR_COUNT; i++)
  {
    if(obs_q1000[i] < baseline)
    {
      baseline = obs_q1000[i];
    }
  }
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    int value = obs_q1000[i] - baseline;
    value = round_div_int(value * LINE_FOLLOW_OBS_CONTRAST_GAIN_Q1000, 1000);
    obs_q1000[i] = clamp_int(value, 0, 1000);
  }
#else
  (void)raw;
  (void)obs_q1000;
#endif
}

static unsigned int ticks_per_us(void)
{
  return (unsigned int)(CLKFREQ / 1000000);
}

static int elapsed_us(unsigned int start_ticks, unsigned int end_ticks)
{
  unsigned int per_us = ticks_per_us();
  unsigned int elapsed_ticks = end_ticks - start_ticks;
  return (int)((elapsed_ticks + per_us / 2) / per_us);
}

static int ticks_to_us(unsigned int ticks)
{
  unsigned int per_us = ticks_per_us();
  return (int)((ticks + per_us / 2) / per_us);
}

static void wait_us_blocking(int wait_us)
{
  if(wait_us > 0)
  {
    waitcnt(CNT + (unsigned int)wait_us * ticks_per_us());
  }
}

static int read_qti_pin(int pin)
{
  high(pin);
  pause(1);
  return rc_time(pin, 1);
}

static void read_qti_grouped_raw(int raw[SENSOR_COUNT])
{
  unsigned int charge_start = CNT;
  unsigned int decay_start;
  unsigned int now;
  unsigned int raw_ticks[SENSOR_COUNT];
  unsigned int timeout_ticks = (unsigned int)QTI_TIMEOUT_US * ticks_per_us();
  int done[SENSOR_COUNT] = {0, 0, 0};
  int done_count = 0;
  int i;

  if(timeout_ticks == 0)
  {
    timeout_ticks = ticks_per_us();
  }

  for(i = 0; i < SENSOR_COUNT; i++)
  {
    raw_ticks[i] = timeout_ticks;
  }

  OUTA |= qti_pin_mask;
  DIRA |= qti_pin_mask;
  wait_us_blocking(QTI_CHARGE_US);
  DIRA &= ~qti_pin_mask;

  decay_start = CNT;
  while(done_count < SENSOR_COUNT)
  {
    unsigned int pins;
    now = CNT;
    if(now - decay_start >= timeout_ticks)
    {
      break;
    }
    pins = INA;
    for(i = 0; i < SENSOR_COUNT; i++)
    {
      if(!done[i] && ((pins & qti_pin_masks[i]) == 0))
      {
        raw_ticks[i] = now - decay_start;
        done[i] = 1;
        done_count++;
      }
    }
  }

  for(i = 0; i < SENSOR_COUNT; i++)
  {
    raw[i] = ticks_to_us(raw_ticks[i]);
  }
  qti_wait_us = elapsed_us(charge_start, CNT);
  qti_sample_us = qti_wait_us;
  qti_sample_period_us = qti_wait_us;
}

#if LINE_FOLLOW_QTI_MAP_MODE != LINE_FOLLOW_QTI_MAP_THREE_PIN
static int max_int_local(int a, int b)
{
  return a > b ? a : b;
}

static void read_qti4_grouped_mapped(int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT])
{
  unsigned int charge_start = CNT;
  unsigned int decay_start;
  unsigned int raw_ticks[QTI_PHYSICAL_COUNT];
  unsigned int timeout_ticks = (unsigned int)QTI_TIMEOUT_US * ticks_per_us();
  int done[QTI_PHYSICAL_COUNT] = {0, 0, 0, 0};
  int raw4[QTI_PHYSICAL_COUNT];
  int obs4[QTI_PHYSICAL_COUNT];
  int done_count = 0;
  int i;

  if(timeout_ticks == 0)
  {
    timeout_ticks = ticks_per_us();
  }
  for(i = 0; i < QTI_PHYSICAL_COUNT; i++)
  {
    raw_ticks[i] = timeout_ticks;
  }

  OUTA |= qti4_pin_mask;
  DIRA |= qti4_pin_mask;
  wait_us_blocking(QTI_CHARGE_US);
  DIRA &= ~qti4_pin_mask;
  decay_start = CNT;

  while(done_count < QTI_PHYSICAL_COUNT)
  {
    unsigned int now = CNT;
    unsigned int pins;
    if(now - decay_start >= timeout_ticks)
    {
      break;
    }
    pins = INA;
    for(i = 0; i < QTI_PHYSICAL_COUNT; i++)
    {
      if(!done[i] && ((pins & qti4_pin_masks[i]) == 0))
      {
        raw_ticks[i] = now - decay_start;
        done[i] = 1;
        done_count++;
      }
    }
  }

  for(i = 0; i < QTI_PHYSICAL_COUNT; i++)
  {
    raw4[i] = ticks_to_us(raw_ticks[i]);
    obs4[i] = normalize_q1000(raw4[i]);
  }
  store_qti_physical_raw(raw4, QTI_PHYSICAL_COUNT);

#if LINE_FOLLOW_QTI_MAP_MODE == LINE_FOLLOW_QTI_MAP_FOUR_ADJACENT_MAX
  obs_q1000[LEFT_SENSOR] = max_int_local(obs4[0], obs4[1]);
  obs_q1000[MIDDLE_SENSOR] = max_int_local(obs4[1], obs4[2]);
  obs_q1000[RIGHT_SENSOR] = max_int_local(obs4[2], obs4[3]);
  raw[LEFT_SENSOR] = max_int_local(raw4[0], raw4[1]);
  raw[MIDDLE_SENSOR] = max_int_local(raw4[1], raw4[2]);
  raw[RIGHT_SENSOR] = max_int_local(raw4[2], raw4[3]);
#else
  obs_q1000[LEFT_SENSOR] = obs4[0];
  obs_q1000[MIDDLE_SENSOR] = max_int_local(obs4[1], obs4[2]);
  obs_q1000[RIGHT_SENSOR] = obs4[3];
  raw[LEFT_SENSOR] = raw4[0];
  raw[MIDDLE_SENSOR] = max_int_local(raw4[1], raw4[2]);
  raw[RIGHT_SENSOR] = raw4[3];
#endif
  qti_wait_us = elapsed_us(charge_start, CNT);
  qti_sample_us = qti_wait_us;
  qti_sample_period_us = qti_wait_us;
}
#endif

#if LINE_FOLLOW_QTI_MODE == LINE_FOLLOW_QTI_MODE_PASM_COG && LINE_FOLLOW_QTI_MAP_MODE == LINE_FOLLOW_QTI_MAP_THREE_PIN
static void start_qti_sampler_cog(void)
{
  int i;
  qti_mailbox.control = 1;
  qti_mailbox.seq = 0;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    qti_mailbox.raw[i] = 0;
    qti_mailbox.pin[i] = qti_pins[i];
  }
  qti_mailbox.sample_us = 0;
  qti_mailbox.period_us = 0;
  qti_mailbox.charge_ticks = QTI_CHARGE_US * (int)ticks_per_us();
  qti_mailbox.timeout_ticks = QTI_TIMEOUT_US * (int)ticks_per_us();
  qti_mailbox.sample_period_ticks = QTI_SAMPLE_PERIOD_US * (int)ticks_per_us();
  qti_mailbox.ticks_per_us = (int)ticks_per_us();
  qti_last_seq = 0;
  qti_cog_id = cognew(_load_start_coguser0, (void*)&qti_mailbox);
  if(qti_cog_id < 0)
  {
    print("QTI PASM cog start failed\n");
    while(1)
    {
      pause(1000);
    }
  }
}

static void stop_qti_sampler_cog(void)
{
  if(qti_cog_id >= 0)
  {
    qti_mailbox.control = 0;
    pause(10);
    cogstop(qti_cog_id);
    qti_cog_id = -1;
  }
}

static void read_qti_cog_raw(int raw[SENSOR_COUNT])
{
  unsigned int wait_start = CNT;
  unsigned int seq_before;
  unsigned int seq_after;
  int stable = 0;
  int i;

  while(!stable)
  {
    do
    {
      seq_before = qti_mailbox.seq;
    }
    while(seq_before == qti_last_seq || (seq_before & 1u));

    for(i = 0; i < SENSOR_COUNT; i++)
    {
      raw[i] = ticks_to_us((unsigned int)qti_mailbox.raw[i]);
    }
    qti_sample_us = ticks_to_us((unsigned int)qti_mailbox.sample_us);
    qti_sample_period_us = ticks_to_us((unsigned int)qti_mailbox.period_us);
    seq_after = qti_mailbox.seq;
    stable = seq_before == seq_after && (seq_after & 1u) == 0;
  }

  qti_last_seq = seq_after;
  qti_wait_us = elapsed_us(wait_start, CNT);
}
#elif LINE_FOLLOW_QTI_MODE == LINE_FOLLOW_QTI_MODE_PASM_COG && LINE_FOLLOW_QTI_MAP_MODE != LINE_FOLLOW_QTI_MAP_THREE_PIN
static void start_qti_sampler_cog(void)
{
  int i;
  qti4_mailbox.control = 1;
  qti4_mailbox.seq = 0;
  for(i = 0; i < QTI_PHYSICAL_COUNT; i++)
  {
    qti4_mailbox.raw[i] = 0;
    qti4_mailbox.pin[i] = qti4_pins[i];
  }
  qti4_mailbox.sample_us = 0;
  qti4_mailbox.period_us = 0;
  qti4_mailbox.charge_ticks = QTI_CHARGE_US * (int)ticks_per_us();
  qti4_mailbox.timeout_ticks = QTI_TIMEOUT_US * (int)ticks_per_us();
  qti4_mailbox.sample_period_ticks = QTI_SAMPLE_PERIOD_US * (int)ticks_per_us();
  qti4_mailbox.ticks_per_us = (int)ticks_per_us();
  qti_last_seq = 0;
  qti_cog_id = cognew(_load_start_coguser0, (void*)&qti4_mailbox);
  if(qti_cog_id < 0)
  {
    print("QTI4 PASM cog start failed\n");
    while(1)
    {
      pause(1000);
    }
  }
}

static void stop_qti_sampler_cog(void)
{
  if(qti_cog_id >= 0)
  {
    qti4_mailbox.control = 0;
    pause(10);
    cogstop(qti_cog_id);
    qti_cog_id = -1;
  }
}

static void read_qti4_cog_mapped(int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT])
{
  unsigned int wait_start = CNT;
  unsigned int seq_before;
  unsigned int seq_after;
  int raw4[QTI_PHYSICAL_COUNT];
  int obs4[QTI_PHYSICAL_COUNT];
  int stable = 0;
  int i;

  while(!stable)
  {
    do
    {
      seq_before = qti4_mailbox.seq;
    }
    while(seq_before == qti_last_seq || (seq_before & 1u));

    for(i = 0; i < QTI_PHYSICAL_COUNT; i++)
    {
      raw4[i] = ticks_to_us((unsigned int)qti4_mailbox.raw[i]);
      obs4[i] = normalize_q1000(raw4[i]);
    }
    qti_sample_us = ticks_to_us((unsigned int)qti4_mailbox.sample_us);
    qti_sample_period_us = ticks_to_us((unsigned int)qti4_mailbox.period_us);
    seq_after = qti4_mailbox.seq;
    stable = seq_before == seq_after && (seq_after & 1u) == 0;
  }
  store_qti_physical_raw(raw4, QTI_PHYSICAL_COUNT);

#if LINE_FOLLOW_QTI_MAP_MODE == LINE_FOLLOW_QTI_MAP_FOUR_ADJACENT_MAX
  obs_q1000[LEFT_SENSOR] = max_int_local(obs4[0], obs4[1]);
  obs_q1000[MIDDLE_SENSOR] = max_int_local(obs4[1], obs4[2]);
  obs_q1000[RIGHT_SENSOR] = max_int_local(obs4[2], obs4[3]);
  raw[LEFT_SENSOR] = max_int_local(raw4[0], raw4[1]);
  raw[MIDDLE_SENSOR] = max_int_local(raw4[1], raw4[2]);
  raw[RIGHT_SENSOR] = max_int_local(raw4[2], raw4[3]);
#else
  obs_q1000[LEFT_SENSOR] = obs4[0];
  obs_q1000[MIDDLE_SENSOR] = max_int_local(obs4[1], obs4[2]);
  obs_q1000[RIGHT_SENSOR] = obs4[3];
  raw[LEFT_SENSOR] = raw4[0];
  raw[MIDDLE_SENSOR] = max_int_local(raw4[1], raw4[2]);
  raw[RIGHT_SENSOR] = raw4[3];
#endif

  qti_last_seq = seq_after;
  qti_wait_us = elapsed_us(wait_start, CNT);
}
#else
static void start_qti_sampler_cog(void)
{
}

static void stop_qti_sampler_cog(void)
{
}
#endif

static void read_qti(int raw[SENSOR_COUNT], int obs_q1000[SENSOR_COUNT],
    float model_obs[OBS_SIZE])
{
  int i;
#if LINE_FOLLOW_QTI_MAP_MODE != LINE_FOLLOW_QTI_MAP_THREE_PIN
#if LINE_FOLLOW_QTI_MODE == LINE_FOLLOW_QTI_MODE_PASM_COG
  read_qti4_cog_mapped(raw, obs_q1000);
#else
  read_qti4_grouped_mapped(raw, obs_q1000);
#endif
  postprocess_obs_q1000(raw, obs_q1000);
#else
#if LINE_FOLLOW_QTI_MODE == LINE_FOLLOW_QTI_MODE_SEQUENTIAL
  unsigned int qti_start = CNT;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    raw[i] = read_qti_pin(qti_pins[i]);
  }
  qti_wait_us = elapsed_us(qti_start, CNT);
  qti_sample_us = qti_wait_us;
  qti_sample_period_us = qti_wait_us;
#elif LINE_FOLLOW_QTI_MODE == LINE_FOLLOW_QTI_MODE_GROUPED_C
  read_qti_grouped_raw(raw);
#else
  read_qti_cog_raw(raw);
#endif
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    obs_q1000[i] = normalize_q1000(raw[i]);
  }
  store_qti_physical_raw(raw, SENSOR_COUNT);
  postprocess_obs_q1000(raw, obs_q1000);
#endif
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
    int action_fixed[NUM_ACTIONS])
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
  }
#endif
}

static int action_fixed_to_q1000(int action_fixed)
{
  return round_div_int(action_fixed * 1000, LINE_FOLLOW_MODEL_FIXED_SCALE);
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

static void print_action_fixed3(int action_fixed)
{
  int scaled = action_fixed_to_q1000(action_fixed);
  if(scaled >= 0)
  {
    print(" ");
  }
  else
  {
    print("-");
    scaled = -scaled;
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

static int float_q1000(float value)
{
  if(value >= 0.0f)
  {
    return (int)(value * 1000.0f + 0.5f);
  }
  return (int)(value * 1000.0f - 0.5f);
}

static void action_q1000_for_log(float actions[NUM_ACTIONS],
    int action_fixed[NUM_ACTIONS], int action_q1000[NUM_ACTIONS])
{
#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FIXED
  action_q1000[0] = action_fixed_to_q1000(action_fixed[0]);
  action_q1000[1] = action_fixed_to_q1000(action_fixed[1]);
#else
  action_q1000[0] = float_q1000(actions[0]);
  action_q1000[1] = float_q1000(actions[1]);
#endif
}

static int log_flags_from_obs_command(const int obs_q1000[SENSOR_COUNT],
    int left_ticks, int right_ticks)
{
  int flags = 0;
  if(obs_q1000[LEFT_SENSOR] >= QTI_THRESHOLD_Q1000) flags |= 1;
  if(obs_q1000[MIDDLE_SENSOR] >= QTI_THRESHOLD_Q1000) flags |= 2;
  if(obs_q1000[RIGHT_SENSOR] >= QTI_THRESHOLD_Q1000) flags |= 4;
  if(obs_q1000[LEFT_SENSOR] >= QTI_THRESHOLD_Q1000
      && obs_q1000[MIDDLE_SENSOR] >= QTI_THRESHOLD_Q1000) flags |= 8;
  if(obs_q1000[MIDDLE_SENSOR] >= QTI_THRESHOLD_Q1000
      && obs_q1000[RIGHT_SENSOR] >= QTI_THRESHOLD_Q1000) flags |= 16;
  if((left_ticks <= MIN_DRIVE_TICKS_PER_SEC)
      != (right_ticks <= MIN_DRIVE_TICKS_PER_SEC)) flags |= 32;
  if(left_ticks <= MIN_DRIVE_TICKS_PER_SEC
      && right_ticks <= MIN_DRIVE_TICKS_PER_SEC) flags |= 64;
  return flags;
}

#if LINE_FOLLOW_SD_LOG
#if LINE_FOLLOW_SD_DEFERRED
static int max_obs_value(const int obs_q1000[SENSOR_COUNT])
{
  int max = obs_q1000[0];
  if(obs_q1000[1] > max) max = obs_q1000[1];
  if(obs_q1000[2] > max) max = obs_q1000[2];
  return max;
}

static int avg_command_ticks(int left_ticks, int right_ticks)
{
  return (left_ticks + right_ticks) / 2;
}

static void deferred_snapshot_fill(DeferredSnapshot* snapshot, int kind,
    int loop, int elapsed_total_ms, int period_us, int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT], int action_q1000[NUM_ACTIONS],
    int left_ticks, int right_ticks, int flags)
{
  int i;
  snapshot->kind = kind;
  snapshot->loop = loop;
  snapshot->elapsed_ms = elapsed_total_ms;
  snapshot->period_us = period_us;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    snapshot->raw[i] = (int16_t)raw[i];
    snapshot->obs_q1000[i] = (int16_t)obs_q1000[i];
  }
  snapshot->action_q1000[0] = (int16_t)action_q1000[0];
  snapshot->action_q1000[1] = (int16_t)action_q1000[1];
  snapshot->command_ticks[0] = (int16_t)left_ticks;
  snapshot->command_ticks[1] = (int16_t)right_ticks;
  snapshot->flags = (int16_t)flags;
}

static void deferred_log_init(DeferredLog* log)
{
  int i;
  log->loops = 0;
  log->final_elapsed_ms = 0;
  log->or_flags = 0;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    log->raw_min[i] = 2147483647;
    log->raw_max[i] = -2147483647;
    log->raw_sum[i] = 0;
    log->obs_min[i] = 2147483647;
    log->obs_max[i] = -2147483647;
    log->obs_sum[i] = 0;
    log->black_count[i] = 0;
  }
  for(i = 0; i < NUM_ACTIONS; i++)
  {
    log->action_min[i] = 2147483647;
    log->action_max[i] = -2147483647;
    log->action_sum[i] = 0;
    log->command_min[i] = 2147483647;
    log->command_max[i] = -2147483647;
    log->command_sum[i] = 0;
    log->prev_command[i] = 0;
  }
  log->period_us_min = 2147483647;
  log->period_us_max = -2147483647;
  log->period_us_sum = 0;
  log->body_us_min = 2147483647;
  log->body_us_max = -2147483647;
  log->body_us_sum = 0;
  log->qti_us_min = 2147483647;
  log->qti_us_max = -2147483647;
  log->qti_us_sum = 0;
  log->policy_us_min = 2147483647;
  log->policy_us_max = -2147483647;
  log->policy_us_sum = 0;
  log->drive_us_min = 2147483647;
  log->drive_us_max = -2147483647;
  log->drive_us_sum = 0;
  log->sample_us_min = 2147483647;
  log->sample_us_max = -2147483647;
  log->sample_us_sum = 0;
  log->all_white_count = 0;
  log->any_black_count = 0;
  log->pair_edge_count = 0;
  log->one_floor_count = 0;
  log->both_floor_count = 0;
  log->command_change_count = 0;
  log->have_prev_command = 0;
  log->strongest_obs = -1;
  log->slowest_line_command = 2147483647;
  log->event_count = 0;
}

static void deferred_log_store_event(DeferredLog* log,
    const DeferredSnapshot* snapshot)
{
  if(log->event_count < LINE_FOLLOW_DEFERRED_EVENT_SLOTS)
  {
    log->events[log->event_count] = *snapshot;
    log->event_count++;
  }
}

static void deferred_log_record(DeferredLog* log, int loop,
    int elapsed_total_ms, int period_us, int body_us, int qti_us,
    int policy_us, int drive_us, int sample_us, int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT], int action_q1000[NUM_ACTIONS],
    int left_ticks, int right_ticks, int flags)
{
  DeferredSnapshot snapshot;
  int i;
  int strongest = max_obs_value(obs_q1000);
  int avg_cmd = avg_command_ticks(left_ticks, right_ticks);
  int command_changed = 0;

  deferred_snapshot_fill(&snapshot, DEFER_EVENT_PERIODIC, loop,
      elapsed_total_ms, period_us, raw, obs_q1000, action_q1000,
      left_ticks, right_ticks, flags);

  if(log->loops == 0)
  {
    snapshot.kind = DEFER_EVENT_FIRST;
    log->first = snapshot;
  }
  snapshot.kind = DEFER_EVENT_LAST;
  log->last = snapshot;

  for(i = 0; i < SENSOR_COUNT; i++)
  {
    if(raw[i] < log->raw_min[i]) log->raw_min[i] = raw[i];
    if(raw[i] > log->raw_max[i]) log->raw_max[i] = raw[i];
    log->raw_sum[i] += raw[i];
    if(obs_q1000[i] < log->obs_min[i]) log->obs_min[i] = obs_q1000[i];
    if(obs_q1000[i] > log->obs_max[i]) log->obs_max[i] = obs_q1000[i];
    log->obs_sum[i] += obs_q1000[i];
  }
  for(i = 0; i < NUM_ACTIONS; i++)
  {
    if(action_q1000[i] < log->action_min[i]) log->action_min[i] = action_q1000[i];
    if(action_q1000[i] > log->action_max[i]) log->action_max[i] = action_q1000[i];
    log->action_sum[i] += action_q1000[i];
  }
  if(left_ticks < log->command_min[0]) log->command_min[0] = left_ticks;
  if(left_ticks > log->command_max[0]) log->command_max[0] = left_ticks;
  if(right_ticks < log->command_min[1]) log->command_min[1] = right_ticks;
  if(right_ticks > log->command_max[1]) log->command_max[1] = right_ticks;
  log->command_sum[0] += left_ticks;
  log->command_sum[1] += right_ticks;

  if(period_us > 0)
  {
    if(period_us < log->period_us_min) log->period_us_min = period_us;
    if(period_us > log->period_us_max) log->period_us_max = period_us;
    log->period_us_sum += period_us;
  }
  if(body_us < log->body_us_min) log->body_us_min = body_us;
  if(body_us > log->body_us_max) log->body_us_max = body_us;
  log->body_us_sum += body_us;
  if(qti_us < log->qti_us_min) log->qti_us_min = qti_us;
  if(qti_us > log->qti_us_max) log->qti_us_max = qti_us;
  log->qti_us_sum += qti_us;
  if(policy_us < log->policy_us_min) log->policy_us_min = policy_us;
  if(policy_us > log->policy_us_max) log->policy_us_max = policy_us;
  log->policy_us_sum += policy_us;
  if(drive_us < log->drive_us_min) log->drive_us_min = drive_us;
  if(drive_us > log->drive_us_max) log->drive_us_max = drive_us;
  log->drive_us_sum += drive_us;
  if(sample_us < log->sample_us_min) log->sample_us_min = sample_us;
  if(sample_us > log->sample_us_max) log->sample_us_max = sample_us;
  log->sample_us_sum += sample_us;

  log->or_flags |= flags;
  if((flags & 7) == 0) log->all_white_count++;
  if(flags & 7) log->any_black_count++;
  if(flags & 1) log->black_count[LEFT_SENSOR]++;
  if(flags & 2) log->black_count[MIDDLE_SENSOR]++;
  if(flags & 4) log->black_count[RIGHT_SENSOR]++;
  if(flags & (8 | 16)) log->pair_edge_count++;
  if(flags & 32) log->one_floor_count++;
  if(flags & 64) log->both_floor_count++;

  if(log->have_prev_command
      && (left_ticks != log->prev_command[0]
          || right_ticks != log->prev_command[1]))
  {
    command_changed = 1;
    log->command_change_count++;
  }
  log->prev_command[0] = left_ticks;
  log->prev_command[1] = right_ticks;
  log->have_prev_command = 1;

  if(strongest > log->strongest_obs)
  {
    snapshot.kind = DEFER_EVENT_STRONGEST;
    log->strongest = snapshot;
    log->strongest_obs = strongest;
  }
  if(strongest >= 100 && avg_cmd < log->slowest_line_command)
  {
    snapshot.kind = DEFER_EVENT_SLOWEST_LINE;
    log->slowest_line = snapshot;
    log->slowest_line_command = avg_cmd;
  }

  if(flags & 64)
  {
    snapshot.kind = DEFER_EVENT_BOTH_FLOOR;
    deferred_log_store_event(log, &snapshot);
  }
  else if(flags & (8 | 16))
  {
    snapshot.kind = DEFER_EVENT_PAIR_EDGE;
    deferred_log_store_event(log, &snapshot);
  }
  else if(command_changed)
  {
    snapshot.kind = DEFER_EVENT_COMMAND_CHANGE;
    deferred_log_store_event(log, &snapshot);
  }
  else if((loop % 128) == 0)
  {
    snapshot.kind = DEFER_EVENT_PERIODIC;
    deferred_log_store_event(log, &snapshot);
  }

  log->loops++;
}
#endif

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

static void backup_previous_log_file(void)
{
#if !LINE_FOLLOW_SD_AUTO_NAME
#if LINE_FOLLOW_SD_BACKUP_PREVIOUS
  FILE* in;
  FILE* out;
  unsigned char bytes[64];
  int total = 0;

  in = fopen(LINE_FOLLOW_SD_FILE, "r");
  if(!in)
  {
    print("SD previous log backup skipped; no %s\n", LINE_FOLLOW_SD_FILE);
    return;
  }

  out = fopen(LINE_FOLLOW_SD_PREV_FILE, "w");
  if(!out)
  {
    fclose(in);
    print("SD previous log backup failed; cannot open %s\n",
        LINE_FOLLOW_SD_PREV_FILE);
    return;
  }

  while(1)
  {
    int count = (int)fread(bytes, 1, sizeof(bytes), in);
    if(count <= 0)
    {
      break;
    }
    if((int)fwrite(bytes, 1, count, out) != count)
    {
      print("SD previous log backup write failed after %d bytes\n", total);
      break;
    }
    total += count;
  }
  fclose(in);
  fclose(out);
  print("SD previous log backed up %s -> %s bytes=%d\n",
      LINE_FOLLOW_SD_FILE, LINE_FOLLOW_SD_PREV_FILE, total);
#endif
#endif
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
#if LINE_FOLLOW_SD_COG_LOG
  header->version = 6;
#elif LINE_FOLLOW_SD_DEFERRED
  header->version = 5;
#else
  header->version = 2;
#endif
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
  header->qti_mode = LINE_FOLLOW_QTI_MODE;
  header->qti_charge_us = QTI_CHARGE_US;
  header->qti_timeout_us = QTI_TIMEOUT_US;
  header->qti_sample_period_us = QTI_SAMPLE_PERIOD_US;
  header->start_delay_ms = LINE_FOLLOW_START_DELAY_MS;
  header->run_ms = LINE_FOLLOW_RUN_MS;
  header->inference_mode = LINE_FOLLOW_INFERENCE_MODE;
  header->build_id = LINE_FOLLOW_BUILD_ID;
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

  backup_previous_log_file();
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

  print("SD logging to %s row_size=%d max_records=%d auto=%d\n",
      sd_log_file, (int)sizeof(SdLogRow), LINE_FOLLOW_SD_MAX_RECORDS,
      LINE_FOLLOW_SD_AUTO_NAME);
  return fp;
}

static void fill_sd_row_struct(SdLogRow* row, int loop, int elapsed_total_ms,
    int period_ms, int body_ms, int period_us, int body_us,
    int qti_us, int policy_us, int drive_us, int sample_us, int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT], float actions[NUM_ACTIONS],
    int action_fixed[NUM_ACTIONS],
    int left_ticks, int right_ticks, int encoder_left_ticks,
    int encoder_right_ticks)
{
  int i;
  int action_q1000[NUM_ACTIONS];

  row->loop = loop;
  row->elapsed_ms = elapsed_total_ms;
  row->period_ms = period_ms;
  row->body_ms = body_ms;
  row->period_us = period_us;
  row->body_us = body_us;
  row->qti_wait_us = qti_us;
  row->policy_us = policy_us;
  row->drive_us = drive_us;
  row->qti_sample_us = sample_us;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    row->raw[i] = raw[i];
    row->obs_q1000[i] = obs_q1000[i];
  }
  row->encoder_left_ticks = encoder_left_ticks;
  row->encoder_right_ticks = encoder_right_ticks;
  action_q1000_for_log(actions, action_fixed, action_q1000);
  row->action_q1000[0] = action_q1000[0];
  row->action_q1000[1] = action_q1000[1];
  row->command_left_ticks = left_ticks;
  row->command_right_ticks = right_ticks;
  row->flags = log_flags_from_obs_command(obs_q1000, left_ticks, right_ticks);
}

static int write_sd_row(FILE* fp, int loop, int elapsed_total_ms,
    int period_ms, int body_ms, int period_us, int body_us,
    int qti_us, int policy_us, int drive_us, int sample_us, int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT], float actions[NUM_ACTIONS],
    int action_fixed[NUM_ACTIONS],
    int left_ticks, int right_ticks, int encoder_left_ticks,
    int encoder_right_ticks)
{
  SdLogRow row;
  fill_sd_row_struct(&row, loop, elapsed_total_ms, period_ms, body_ms,
      period_us, body_us, qti_us, policy_us, drive_us, sample_us,
      raw, obs_q1000, actions, action_fixed, left_ticks, right_ticks,
      encoder_left_ticks, encoder_right_ticks);

  return fwrite(&row, sizeof(row), 1, fp) == 1 ? 0 : -1;
}

static void close_sd_log(FILE** fp, int records, const char* reason)
{
  if(*fp)
  {
    fflush(*fp);
    fclose(*fp);
    *fp = 0;
    if(records > 0)
    {
      write_last_log_file(sd_log_file);
    }
    print("SD log closed records=%d reason=%s\n", records, reason);
  }
}

#if LINE_FOLLOW_SD_COG_LOG
static void compact_trace_init(void)
{
  int i;
  trace_count = 0;
  trace_seen = 0;
  trace_dropped = 0;
  trace_last_left = -32768;
  trace_last_right = -32768;
  trace_or_flags = 0;
  trace_all_white_count = 0;
  trace_any_black_count = 0;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    trace_raw_min[i] = 2147483647;
    trace_raw_max[i] = -2147483647;
    trace_obs_min[i] = 2147483647;
    trace_obs_max[i] = -2147483647;
  }
  for(i = 0; i < NUM_ACTIONS; i++)
  {
    trace_command_min[i] = 2147483647;
    trace_command_max[i] = -2147483647;
  }
}

static void compact_trace_store(int kind, int loop, int elapsed_total_ms,
    int period_us, int raw[SENSOR_COUNT], int obs_q1000[SENSOR_COUNT],
    int action_q1000[NUM_ACTIONS], int left_ticks, int right_ticks, int flags)
{
  CompactTraceRow* row;
  int i;
  if(trace_count >= LINE_FOLLOW_TRACE_SLOTS)
  {
    trace_dropped++;
    return;
  }
  row = &trace_rows[trace_count++];
  row->kind = (int16_t)kind;
  row->loop = loop;
  row->elapsed_ms = elapsed_total_ms;
  row->period_us = period_us;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    row->raw[i] = (int16_t)raw[i];
    row->obs_q1000[i] = (int16_t)obs_q1000[i];
  }
  row->action_q1000[0] = (int16_t)action_q1000[0];
  row->action_q1000[1] = (int16_t)action_q1000[1];
  row->command_ticks[0] = (int16_t)left_ticks;
  row->command_ticks[1] = (int16_t)right_ticks;
  row->flags = (int16_t)flags;
}

static void compact_trace_record(int loop, int elapsed_total_ms,
    int period_us, int raw[SENSOR_COUNT], int obs_q1000[SENSOR_COUNT],
    int action_q1000[NUM_ACTIONS], int left_ticks, int right_ticks, int flags)
{
  int i;
  int should_store = 0;
  int kind = 1;

  trace_seen++;
  trace_or_flags |= flags;
  if((flags & 7) == 0) trace_all_white_count++;
  if(flags & 7) trace_any_black_count++;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    if(raw[i] < trace_raw_min[i]) trace_raw_min[i] = raw[i];
    if(raw[i] > trace_raw_max[i]) trace_raw_max[i] = raw[i];
    if(obs_q1000[i] < trace_obs_min[i]) trace_obs_min[i] = obs_q1000[i];
    if(obs_q1000[i] > trace_obs_max[i]) trace_obs_max[i] = obs_q1000[i];
  }
  if(left_ticks < trace_command_min[0]) trace_command_min[0] = left_ticks;
  if(left_ticks > trace_command_max[0]) trace_command_max[0] = left_ticks;
  if(right_ticks < trace_command_min[1]) trace_command_min[1] = right_ticks;
  if(right_ticks > trace_command_max[1]) trace_command_max[1] = right_ticks;

  if(loop == 0)
  {
    should_store = 1;
    kind = 1;
  }
  else if(left_ticks != trace_last_left || right_ticks != trace_last_right)
  {
    should_store = 1;
    kind = 2;
  }
  else if(flags & 64)
  {
    should_store = 1;
    kind = 3;
  }
  else if(flags & (8 | 16))
  {
    should_store = 1;
    kind = 4;
  }
  else if(LINE_FOLLOW_TRACE_PERIOD > 0 && (loop % LINE_FOLLOW_TRACE_PERIOD) == 0)
  {
    should_store = 1;
    kind = 5;
  }

  if(should_store)
  {
    compact_trace_store(kind, loop, elapsed_total_ms, period_us, raw,
        obs_q1000, action_q1000, left_ticks, right_ticks, flags);
  }
  trace_last_left = left_ticks;
  trace_last_right = right_ticks;
}

static void compact_clear_sd_row(SdLogRow* row)
{
  int i;
  row->loop = 0;
  row->elapsed_ms = 0;
  row->period_ms = 0;
  row->body_ms = 0;
  row->period_us = 0;
  row->body_us = 0;
  row->qti_wait_us = 0;
  row->policy_us = 0;
  row->drive_us = 0;
  row->qti_sample_us = 0;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    row->raw[i] = 0;
    row->obs_q1000[i] = 0;
  }
  row->encoder_left_ticks = 0;
  row->encoder_right_ticks = 0;
  row->action_q1000[0] = 0;
  row->action_q1000[1] = 0;
  row->command_left_ticks = 0;
  row->command_right_ticks = 0;
  row->flags = 0;
}

static int compact_write_sd_row(FILE* fp, SdLogRow* row)
{
  return fwrite(row, sizeof(*row), 1, fp) == 1 ? 0 : -1;
}

static int compact_write_stats_row(FILE* fp, int kind)
{
  SdLogRow row;
  compact_clear_sd_row(&row);
  row.loop = -kind;
  row.encoder_left_ticks = kind;
  if(kind == 1)
  {
    row.raw[0] = trace_seen;
    row.raw[1] = trace_count;
    row.raw[2] = trace_dropped;
    row.obs_q1000[0] = trace_all_white_count;
    row.obs_q1000[1] = trace_any_black_count;
    row.flags = trace_or_flags;
  }
  else if(kind == 2)
  {
    row.raw[0] = trace_raw_min[0] == 2147483647 ? 0 : trace_raw_min[0];
    row.raw[1] = trace_raw_min[1] == 2147483647 ? 0 : trace_raw_min[1];
    row.raw[2] = trace_raw_min[2] == 2147483647 ? 0 : trace_raw_min[2];
    row.obs_q1000[0] = trace_obs_min[0] == 2147483647 ? 0 : trace_obs_min[0];
    row.obs_q1000[1] = trace_obs_min[1] == 2147483647 ? 0 : trace_obs_min[1];
    row.obs_q1000[2] = trace_obs_min[2] == 2147483647 ? 0 : trace_obs_min[2];
    row.command_left_ticks = trace_command_min[0] == 2147483647 ? 0 : trace_command_min[0];
    row.command_right_ticks = trace_command_min[1] == 2147483647 ? 0 : trace_command_min[1];
  }
  else
  {
    row.raw[0] = trace_raw_max[0] < 0 ? 0 : trace_raw_max[0];
    row.raw[1] = trace_raw_max[1] < 0 ? 0 : trace_raw_max[1];
    row.raw[2] = trace_raw_max[2] < 0 ? 0 : trace_raw_max[2];
    row.obs_q1000[0] = trace_obs_max[0] < 0 ? 0 : trace_obs_max[0];
    row.obs_q1000[1] = trace_obs_max[1] < 0 ? 0 : trace_obs_max[1];
    row.obs_q1000[2] = trace_obs_max[2] < 0 ? 0 : trace_obs_max[2];
    row.command_left_ticks = trace_command_max[0] < 0 ? 0 : trace_command_max[0];
    row.command_right_ticks = trace_command_max[1] < 0 ? 0 : trace_command_max[1];
  }
  return compact_write_sd_row(fp, &row);
}

static int compact_write_trace_row(FILE* fp, const CompactTraceRow* trace)
{
  SdLogRow row;
  int i;
  compact_clear_sd_row(&row);
  row.loop = trace->loop;
  row.elapsed_ms = trace->elapsed_ms;
  row.period_us = trace->period_us;
  row.encoder_left_ticks = trace->kind;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    row.raw[i] = trace->raw[i];
    row.obs_q1000[i] = trace->obs_q1000[i];
  }
  row.action_q1000[0] = trace->action_q1000[0];
  row.action_q1000[1] = trace->action_q1000[1];
  row.command_left_ticks = trace->command_ticks[0];
  row.command_right_ticks = trace->command_ticks[1];
  row.flags = trace->flags;
  return compact_write_sd_row(fp, &row);
}

static int compact_write_log_to_sd(void)
{
  FILE* fp = open_sd_log();
  int records = 0;
  int i;
  if(!fp) return -1;
  if(compact_write_stats_row(fp, 1)) goto fail;
  records++;
  if(compact_write_stats_row(fp, 2)) goto fail;
  records++;
  if(compact_write_stats_row(fp, 3)) goto fail;
  records++;
  for(i = 0; i < trace_count; i++)
  {
    if(compact_write_trace_row(fp, &trace_rows[i])) goto fail;
    records++;
  }
  close_sd_log(&fp, records, "postrun_cog");
  return records;
fail:
  close_sd_log(&fp, records, "postrun_cog_failed");
  return -1;
}

static void sd_cog_postrun_main(void* par)
{
  (void)par;
  sd_cog_mailbox.status = SD_COG_WRITING;
  sd_cog_mailbox.records = compact_write_log_to_sd();
  sd_cog_mailbox.status = sd_cog_mailbox.records >= 0 ? SD_COG_DONE : SD_COG_ERROR;
  while(1) pause(1000);
}

static void start_postrun_sd_cog(void)
{
  sd_cog_mailbox.status = SD_COG_IDLE;
  sd_cog_mailbox.records = 0;
  sd_cog_mailbox.dropped = trace_dropped;
  sd_cog_id = cogstart(sd_cog_postrun_main, 0,
      sd_cog_stack, sizeof(sd_cog_stack));
  if(sd_cog_id < 0)
  {
    sd_cog_mailbox.status = SD_COG_ERROR;
    print("SD postrun cog start failed\n");
  }
}

static void wait_postrun_sd_cog(void)
{
  int waited_ms = 0;
  if(sd_cog_id < 0) return;
  while(sd_cog_mailbox.status == SD_COG_IDLE
      || sd_cog_mailbox.status == SD_COG_WRITING)
  {
    pause(100);
    waited_ms += 100;
    if(LINE_FOLLOW_SD_WRITE_TIMEOUT_MS > 0
        && waited_ms >= LINE_FOLLOW_SD_WRITE_TIMEOUT_MS)
    {
      print("SD postrun cog timeout records=%d trace=%d dropped=%d\n",
          sd_cog_mailbox.records, trace_count, trace_dropped);
      cogstop(sd_cog_id);
      sd_cog_id = -1;
      sd_cog_mailbox.status = SD_COG_ERROR;
      return;
    }
  }
  print("SD postrun cog status=%d records=%d trace=%d dropped=%d\n",
      sd_cog_mailbox.status, sd_cog_mailbox.records, trace_count, trace_dropped);
  cogstop(sd_cog_id);
  sd_cog_id = -1;
}

#if 0
static void sd_cog_publish_row(int loop, int elapsed_total_ms,
    int period_ms, int body_ms, int period_us, int body_us,
    int qti_us, int policy_us, int drive_us, int sample_us, int raw[SENSOR_COUNT],
    int obs_q1000[SENSOR_COUNT], float actions[NUM_ACTIONS],
    int action_fixed[NUM_ACTIONS],
    int left_ticks, int right_ticks, int encoder_left_ticks,
    int encoder_right_ticks)
{
  unsigned int seq = sd_cog_mailbox.seq;
  sd_cog_mailbox.seq = seq | 1u;
  fill_sd_row_struct((SdLogRow*)&sd_cog_mailbox.row, loop, elapsed_total_ms,
      period_ms, body_ms, period_us, body_us, qti_us, policy_us, drive_us,
      sample_us, raw, obs_q1000, actions, action_fixed, left_ticks,
      right_ticks, encoder_left_ticks, encoder_right_ticks);
  sd_cog_mailbox.seq = (seq + 2u) & ~1u;
}

static void sd_cog_logger_main(void* par)
{
  FILE* fp;
  SdLogRow row;
  unsigned int last_seq = 0;
  (void)par;

  sd_cog_mailbox.status = SD_COG_OPENING;
  fp = open_sd_log();
  if(!fp)
  {
    sd_cog_mailbox.status = SD_COG_ERROR;
    while(sd_cog_mailbox.control)
    {
      pause(100);
    }
    return;
  }
  sd_cog_mailbox.status = SD_COG_WRITING;

  while(sd_cog_mailbox.control)
  {
    unsigned int seq_before = sd_cog_mailbox.seq;
    unsigned int seq_after;
    if(seq_before == last_seq || (seq_before & 1u))
    {
      pause(1);
      continue;
    }
    row = sd_cog_mailbox.row;
    seq_after = sd_cog_mailbox.seq;
    if(seq_before != seq_after || (seq_after & 1u))
    {
      continue;
    }
    if(last_seq != 0 && seq_after != last_seq + 2u)
    {
      sd_cog_mailbox.dropped++;
    }
    if(fwrite(&row, sizeof(row), 1, fp) != 1)
    {
      sd_cog_mailbox.status = SD_COG_ERROR;
      break;
    }
    sd_cog_mailbox.records++;
    last_seq = seq_after;
    if(LINE_FOLLOW_SD_FLUSH_EVERY > 0
        && (sd_cog_mailbox.records % LINE_FOLLOW_SD_FLUSH_EVERY) == 0)
    {
      fflush(fp);
    }
    if(LINE_FOLLOW_SD_MAX_RECORDS > 0
        && sd_cog_mailbox.records >= LINE_FOLLOW_SD_MAX_RECORDS)
    {
      break;
    }
  }

  close_sd_log(&fp, sd_cog_mailbox.records, "sd_cog");
  if(sd_cog_mailbox.status != SD_COG_ERROR)
  {
    sd_cog_mailbox.status = SD_COG_DONE;
  }
  while(1)
  {
    pause(1000);
  }
}

static void start_sd_cog_logger(void)
{
  sd_cog_mailbox.control = 1;
  sd_cog_mailbox.seq = 0;
  sd_cog_mailbox.status = SD_COG_IDLE;
  sd_cog_mailbox.records = 0;
  sd_cog_mailbox.dropped = 0;
  sd_cog_id = cogstart(sd_cog_logger_main, 0,
      sd_cog_stack, sizeof(sd_cog_stack));
  if(sd_cog_id < 0)
  {
    sd_cog_mailbox.status = SD_COG_ERROR;
    print("SD cog logger start failed\n");
  }
}

static void stop_sd_cog_logger(void)
{
  int waited_ms = 0;
  if(sd_cog_id < 0)
  {
    return;
  }
  sd_cog_mailbox.control = 0;
  while(sd_cog_mailbox.status == SD_COG_OPENING
      || sd_cog_mailbox.status == SD_COG_WRITING)
  {
    pause(100);
    waited_ms += 100;
    if(LINE_FOLLOW_SD_WRITE_TIMEOUT_MS > 0
        && waited_ms >= LINE_FOLLOW_SD_WRITE_TIMEOUT_MS)
    {
      print("SD cog logger stop timeout records=%d dropped=%d status=%d\n",
          sd_cog_mailbox.records, sd_cog_mailbox.dropped,
          sd_cog_mailbox.status);
      cogstop(sd_cog_id);
      sd_cog_id = -1;
      sd_cog_mailbox.status = SD_COG_ERROR;
      return;
    }
  }
  print("SD cog logger status=%d records=%d dropped=%d\n",
      sd_cog_mailbox.status, sd_cog_mailbox.records,
      sd_cog_mailbox.dropped);
  cogstop(sd_cog_id);
  sd_cog_id = -1;
}
#endif
#endif

#if LINE_FOLLOW_SD_DEFERRED
static void clear_sd_row(SdLogRow* row)
{
  int i;
  row->loop = 0;
  row->elapsed_ms = 0;
  row->period_ms = 0;
  row->body_ms = 0;
  row->period_us = 0;
  row->body_us = 0;
  row->qti_wait_us = 0;
  row->policy_us = 0;
  row->drive_us = 0;
  row->qti_sample_us = 0;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    row->raw[i] = 0;
    row->obs_q1000[i] = 0;
  }
  row->encoder_left_ticks = 0;
  row->encoder_right_ticks = 0;
  row->action_q1000[0] = 0;
  row->action_q1000[1] = 0;
  row->command_left_ticks = 0;
  row->command_right_ticks = 0;
  row->flags = 0;
}

static int write_deferred_row(FILE* fp, const SdLogRow* row)
{
  return fwrite(row, sizeof(*row), 1, fp) == 1 ? 0 : -1;
}

static int write_deferred_snapshot(FILE* fp, const DeferredSnapshot* snapshot)
{
  SdLogRow row;
  int i;
  clear_sd_row(&row);
  row.loop = snapshot->loop;
  row.elapsed_ms = snapshot->elapsed_ms;
  row.period_us = snapshot->period_us;
  for(i = 0; i < SENSOR_COUNT; i++)
  {
    row.raw[i] = snapshot->raw[i];
    row.obs_q1000[i] = snapshot->obs_q1000[i];
  }
  row.encoder_left_ticks = snapshot->kind;
  row.action_q1000[0] = snapshot->action_q1000[0];
  row.action_q1000[1] = snapshot->action_q1000[1];
  row.command_left_ticks = snapshot->command_ticks[0];
  row.command_right_ticks = snapshot->command_ticks[1];
  row.flags = snapshot->flags;
  return write_deferred_row(fp, &row);
}

static int write_deferred_stats(FILE* fp, const DeferredLog* log, int kind)
{
  SdLogRow row;
  int i;
  int loops = log->loops > 0 ? log->loops : 1;
  clear_sd_row(&row);
  row.loop = -kind;
  row.elapsed_ms = log->final_elapsed_ms;
  row.encoder_left_ticks = kind;
  if(kind == 1)
  {
    row.period_us = log->period_us_min == 2147483647 ? 0 : log->period_us_min;
    row.body_us = log->body_us_min == 2147483647 ? 0 : log->body_us_min;
    row.qti_wait_us = log->qti_us_min == 2147483647 ? 0 : log->qti_us_min;
    row.policy_us = log->policy_us_min == 2147483647 ? 0 : log->policy_us_min;
    row.drive_us = log->drive_us_min == 2147483647 ? 0 : log->drive_us_min;
    row.qti_sample_us = log->sample_us_min == 2147483647 ? 0 : log->sample_us_min;
    for(i = 0; i < SENSOR_COUNT; i++)
    {
      row.raw[i] = log->raw_min[i] == 2147483647 ? 0 : log->raw_min[i];
      row.obs_q1000[i] = log->obs_min[i] == 2147483647 ? 0 : log->obs_min[i];
    }
    row.action_q1000[0] = log->action_min[0] == 2147483647 ? 0 : log->action_min[0];
    row.action_q1000[1] = log->action_min[1] == 2147483647 ? 0 : log->action_min[1];
    row.command_left_ticks = log->command_min[0] == 2147483647 ? 0 : log->command_min[0];
    row.command_right_ticks = log->command_min[1] == 2147483647 ? 0 : log->command_min[1];
  }
  else if(kind == 2)
  {
    row.period_us = log->period_us_max < 0 ? 0 : log->period_us_max;
    row.body_us = log->body_us_max < 0 ? 0 : log->body_us_max;
    row.qti_wait_us = log->qti_us_max < 0 ? 0 : log->qti_us_max;
    row.policy_us = log->policy_us_max < 0 ? 0 : log->policy_us_max;
    row.drive_us = log->drive_us_max < 0 ? 0 : log->drive_us_max;
    row.qti_sample_us = log->sample_us_max < 0 ? 0 : log->sample_us_max;
    for(i = 0; i < SENSOR_COUNT; i++)
    {
      row.raw[i] = log->raw_max[i] < 0 ? 0 : log->raw_max[i];
      row.obs_q1000[i] = log->obs_max[i] < 0 ? 0 : log->obs_max[i];
    }
    row.action_q1000[0] = log->action_max[0] < -2000000000 ? 0 : log->action_max[0];
    row.action_q1000[1] = log->action_max[1] < -2000000000 ? 0 : log->action_max[1];
    row.command_left_ticks = log->command_max[0] < -2000000000 ? 0 : log->command_max[0];
    row.command_right_ticks = log->command_max[1] < -2000000000 ? 0 : log->command_max[1];
  }
  else if(kind == 3)
  {
    row.period_us = log->period_us_sum / loops;
    row.body_us = log->body_us_sum / loops;
    row.qti_wait_us = log->qti_us_sum / loops;
    row.policy_us = log->policy_us_sum / loops;
    row.drive_us = log->drive_us_sum / loops;
    row.qti_sample_us = log->sample_us_sum / loops;
    for(i = 0; i < SENSOR_COUNT; i++)
    {
      row.raw[i] = log->raw_sum[i] / loops;
      row.obs_q1000[i] = log->obs_sum[i] / loops;
    }
    row.action_q1000[0] = log->action_sum[0] / loops;
    row.action_q1000[1] = log->action_sum[1] / loops;
    row.command_left_ticks = log->command_sum[0] / loops;
    row.command_right_ticks = log->command_sum[1] / loops;
  }
  else
  {
    row.raw[0] = log->loops;
    row.raw[1] = log->all_white_count;
    row.raw[2] = log->any_black_count;
    row.obs_q1000[0] = log->black_count[LEFT_SENSOR];
    row.obs_q1000[1] = log->black_count[MIDDLE_SENSOR];
    row.obs_q1000[2] = log->black_count[RIGHT_SENSOR];
    row.action_q1000[0] = log->command_change_count;
    row.action_q1000[1] = log->event_count;
    row.command_left_ticks = log->both_floor_count;
    row.command_right_ticks = log->one_floor_count;
    row.period_us = log->pair_edge_count;
    row.flags = log->or_flags;
  }
  return write_deferred_row(fp, &row);
}

static int write_deferred_sd_log(const DeferredLog* log)
{
  FILE* fp = open_sd_log();
  int records = 0;
  int i;
  if(!fp)
  {
    return -1;
  }
  if(write_deferred_stats(fp, log, 1)) goto fail;
  records++;
  if(write_deferred_stats(fp, log, 2)) goto fail;
  records++;
  if(write_deferred_stats(fp, log, 3)) goto fail;
  records++;
  if(write_deferred_stats(fp, log, 4)) goto fail;
  records++;
  if(log->loops > 0)
  {
    if(write_deferred_snapshot(fp, &log->first)) goto fail;
    records++;
    if(write_deferred_snapshot(fp, &log->last)) goto fail;
    records++;
    if(log->strongest_obs >= 0)
    {
      if(write_deferred_snapshot(fp, &log->strongest)) goto fail;
      records++;
    }
    if(log->slowest_line_command != 2147483647)
    {
      if(write_deferred_snapshot(fp, &log->slowest_line)) goto fail;
      records++;
    }
    for(i = 0; i < log->event_count; i++)
    {
      if(write_deferred_snapshot(fp, &log->events[i])) goto fail;
      records++;
    }
  }
  close_sd_log(&fp, records, "logger_cog");
  return records;

fail:
  print("SD deferred write failed records=%d\n", records);
  close_sd_log(&fp, records, "logger_cog_failed");
  return -1;
}

static void sd_logger_main(void* par)
{
  (void)par;
  while(sd_logger_state != LOGGER_REQUESTED)
  {
    pause(10);
  }
  sd_logger_state = LOGGER_WRITING;
  sd_logger_records = write_deferred_sd_log(&deferred_log);
  sd_logger_state = sd_logger_records >= 0 ? LOGGER_DONE : LOGGER_ERROR;
  while(1)
  {
    pause(1000);
  }
}
#endif

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

static void blink_done_forever(void)
{
  while(1)
  {
    high(26);
    low(27);
    pause(150);
    low(26);
    high(27);
    pause(150);
  }
}

#ifndef LINE_FOLLOW_MODEL_LIVE_NO_MAIN
int main(void)
{
  int raw[SENSOR_COUNT];
  int obs_q1000[SENSOR_COUNT];
  int raw_min[SENSOR_COUNT] = {2147483647, 2147483647, 2147483647};
  int raw_max[SENSOR_COUNT] = {0, 0, 0};
  float model_obs[OBS_SIZE];
  float actions[NUM_ACTIONS];
  int action_fixed[NUM_ACTIONS] = {0, 0};
  int action_q1000[NUM_ACTIONS] = {0, 0};
  int log_flags = 0;
  int left_ticks = 0;
  int right_ticks = 0;
  int loops = 0;
  unsigned int loop_start_ticks = 0;
  unsigned int prev_loop_start_ticks = 0;
  unsigned int run_start_ticks = 0;
  unsigned int qti_end_ticks = 0;
  unsigned int policy_end_ticks = 0;
  unsigned int drive_end_ticks = 0;
  unsigned int loop_end_ticks = 0;
  int body_ms = 0;
  int period_ms = 0;
  int body_us = 0;
  int period_us = 0;
  int policy_us = 0;
  int drive_us = 0;
  int elapsed_total_ms = 0;
  int i;
#if LINE_FOLLOW_SD_LOG
  FILE* sd_fp = 0;
  int sd_records = 0;
  int sd_total_records = 0;
  int sd_failed = 0;
  int sd_enabled = 0;
  int encoder_left_ticks = 0;
  int encoder_right_ticks = 0;
#endif

  low(26);
  low(27);
  if((LINE_FOLLOW_MODEL_RAW_FLOATS != expected_raw_float_count()
        && LINE_FOLLOW_MODEL_RAW_FLOATS != expected_aligned_read_float_count())
      || LINE_FOLLOW_MODEL_OBS_SIZE != OBS_SIZE
      || LINE_FOLLOW_MODEL_HIDDEN_SIZE != HIDDEN_SIZE
      || LINE_FOLLOW_MODEL_NUM_LAYERS != NUM_LAYERS
      || LINE_FOLLOW_MODEL_NUM_ACTIONS != NUM_ACTIONS)
  {
    print("\nLINE_FOLLOW_MODEL_LIVE shape mismatch\n");
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

#if LINE_FOLLOW_STARTUP_PRINTS
  print("\nLINE_FOLLOW_MODEL_LIVE v1 drive=%d\n", LINE_FOLLOW_ENABLE_DRIVE);
  print("checkpoint=%s\n", line_follow_model_checkpoint);
  print("raw_floats=%d padded_floats=%d\n",
      LINE_FOLLOW_MODEL_RAW_FLOATS, LINE_FOLLOW_MODEL_PADDED_FLOATS);
  print("hidden_size=%d num_layers=%d inference_mode=%d folded_supported=%d fixed_scale=%d\n",
      HIDDEN_SIZE, NUM_LAYERS, LINE_FOLLOW_INFERENCE_MODE,
      LINE_FOLLOW_MODEL_FOLDED_SUPPORTED, LINE_FOLLOW_MODEL_FIXED_SCALE);
  print("pins left=%d middle=%d right=%d\n",
      QTI_LEFT_PIN, QTI_MIDDLE_PIN, QTI_RIGHT_PIN);
  print("qti_map_mode=%d far_right_pin=%d\n",
      LINE_FOLLOW_QTI_MAP_MODE, QTI_FAR_RIGHT_PIN);
  print("cal white=%d black=%d threshold_q1000=%d loop_ms=%d max_loops=%d run_ms=%d print_every=%d max_speed_mm_s=%d deadband_q1000=%d min_ticks=%d\n",
      QTI_WHITE_TIME, QTI_BLACK_TIME, QTI_THRESHOLD_Q1000,
      LINE_FOLLOW_LOOP_MS, LINE_FOLLOW_MAX_LOOPS, LINE_FOLLOW_RUN_MS,
      LINE_FOLLOW_PRINT_EVERY,
      (int)(MAX_WHEEL_SPEED_MPS * 1000.0f + 0.5f),
      (int)(COMMAND_DEADBAND * 1000.0f + 0.5f),
      (int)(MIN_DRIVE_TICKS_PER_SEC));
  print("obs_subtract_min=%d obs_contrast_gain_q1000=%d\n",
      LINE_FOLLOW_OBS_SUBTRACT_MIN, LINE_FOLLOW_OBS_CONTRAST_GAIN_Q1000);
  print("obs_raw_diff=%d obs_raw_diff_gain_q1000=%d obs_raw_diff_span=%d\n",
      LINE_FOLLOW_OBS_RAW_DIFF, LINE_FOLLOW_OBS_RAW_DIFF_GAIN_Q1000,
      LINE_FOLLOW_OBS_RAW_DIFF_SPAN);
  print("qti_mode=%d charge_us=%d timeout_us=%d sample_period_us=%d\n",
      LINE_FOLLOW_QTI_MODE, QTI_CHARGE_US, QTI_TIMEOUT_US,
      QTI_SAMPLE_PERIOD_US);
  print("sd_log=%d sd_file=%s sd_max_records=%d sd_log_every=%d sd_flush_every=%d sd_chunk_records=%d\n",
      LINE_FOLLOW_SD_LOG, LINE_FOLLOW_SD_FILE, LINE_FOLLOW_SD_MAX_RECORDS,
      LINE_FOLLOW_SD_LOG_EVERY, LINE_FOLLOW_SD_FLUSH_EVERY,
      LINE_FOLLOW_SD_CHUNK_RECORDS);
  print("sd_auto_name=%d sd_last_file=%s\n",
      LINE_FOLLOW_SD_AUTO_NAME, SD_LOG_LAST_FILE);
  print("start_delay_ms=%d\n", LINE_FOLLOW_START_DELAY_MS);
  print("clock clkfreq=%d ms_ticks=%d\n", CLKFREQ, ms);
  print("NOTE lower raw should be brighter/whiter, higher raw should be darker/blacker\n");
  print("model observations are left, middle, right");
#if LINE_FOLLOW_QTI_MAP_MODE == LINE_FOLLOW_QTI_MAP_THREE_PIN
  print("; pin 4 is ignored\n");
#elif LINE_FOLLOW_QTI_MAP_MODE == LINE_FOLLOW_QTI_MAP_FOUR_ADJACENT_MAX
  print("; four physical sensors are adjacent-max pooled\n");
#else
  print("; four physical sensors are center-max pooled\n");
#endif
#if LINE_FOLLOW_QTI_MAP_MODE != LINE_FOLLOW_QTI_MAP_THREE_PIN
  print("physical_pins phys0=%d phys1=%d phys2=%d phys3=%d\n",
      qti4_pins[0], qti4_pins[1], qti4_pins[2], qti4_pins[3]);
  print("P step phys0 phys1 phys2 phys3\n");
#endif
  print("L step raw0 raw1 raw2 min0 min1 min2 max0 max1 max2 sensor0 sensor1 sensor2 model0 model1 model2 action0 action1 left right dt_ms period_us body_us qti_us policy_us drive_us qti_sample_us\n");
#endif

  start_qti_sampler_cog();

#if LINE_FOLLOW_ENABLE_DRIVE
  drive_setRampStep(4);
  drive_speed(0, 0);
#endif

  if(LINE_FOLLOW_START_DELAY_MS > 0)
  {
    int elapsed = 0;
#if LINE_FOLLOW_STARTUP_PRINTS
    print("Start delay: place robot on track now.\n");
#endif
    while(elapsed < LINE_FOLLOW_START_DELAY_MS)
    {
      high(26);
      low(27);
      pause(125);
      low(26);
      high(27);
      pause(125);
      elapsed += 250;
    }
    low(26);
    low(27);
  }

#if LINE_FOLLOW_SD_LOG
#if LINE_FOLLOW_SD_COG_LOG
  compact_trace_init();
#elif LINE_FOLLOW_SD_DEFERRED
  deferred_log_init(&deferred_log);
#else
  sd_fp = open_sd_log();
  if(!sd_fp)
  {
    sd_failed = 1;
    if(LINE_FOLLOW_SD_REQUIRED)
    {
      print("SD logging is required for this build; motors will not start.\n");
      blink_sd_error_forever();
    }
    print("SD logging unavailable; continuing drive without SD log.\n");
  }
  else
  {
    sd_enabled = 1;
  }
#endif
#endif

#if LINE_FOLLOW_ENABLE_DRIVE
  pause(500);
#endif

  while(LINE_FOLLOW_MAX_LOOPS == 0 || loops < LINE_FOLLOW_MAX_LOOPS)
  {
    if(LINE_FOLLOW_RUN_MS > 0 && run_start_ticks != 0
        && elapsed_ms(run_start_ticks, read_clock_ticks()) >= LINE_FOLLOW_RUN_MS)
    {
      break;
    }
#if LINE_FOLLOW_SD_LOG
#if !LINE_FOLLOW_SD_DEFERRED && !LINE_FOLLOW_SD_COG_LOG
    if(LINE_FOLLOW_SD_STOP_ON_WRITE_FAIL && sd_failed) break;
    if(sd_enabled && LINE_FOLLOW_SD_CHUNK_RECORDS > 0
        && sd_records >= LINE_FOLLOW_SD_CHUNK_RECORDS)
    {
      close_sd_log(&sd_fp, sd_records, "chunk_records");
      sd_records = 0;
      sd_enabled = 0;
    }
    if(sd_enabled && LINE_FOLLOW_SD_MAX_RECORDS > 0
        && sd_total_records >= LINE_FOLLOW_SD_MAX_RECORDS)
    {
      close_sd_log(&sd_fp, sd_records, "max_records");
      sd_records = 0;
      sd_enabled = 0;
    }
    if(!sd_enabled && !sd_failed && LINE_FOLLOW_SD_CHUNK_RECORDS > 0
        && (LINE_FOLLOW_SD_MAX_RECORDS == 0
            || sd_total_records < LINE_FOLLOW_SD_MAX_RECORDS))
    {
      sd_fp = open_sd_log();
      if(!sd_fp)
      {
        sd_failed = 1;
        if(LINE_FOLLOW_SD_REQUIRED)
        {
          print("SD logging is required for this build; motors will stop.\n");
          break;
        }
      }
      else
      {
        sd_enabled = 1;
      }
    }
#endif
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
    period_us = prev_loop_start_ticks == 0
        ? 0
        : elapsed_us(prev_loop_start_ticks, loop_start_ticks);
    prev_loop_start_ticks = loop_start_ticks;
    read_qti(raw, obs_q1000, model_obs);
    qti_end_ticks = read_clock_ticks();
    for(i = 0; i < SENSOR_COUNT; i++)
    {
      if(raw[i] < raw_min[i]) raw_min[i] = raw[i];
      if(raw[i] > raw_max[i]) raw_max[i] = raw[i];
    }

#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FIXED
    forward_model_fixed(obs_q1000, action_fixed);
    left_ticks = action_fixed_to_ticks(action_fixed[0]);
    right_ticks = action_fixed_to_ticks(action_fixed[1]);
#else
    forward_model(model_obs, actions);
    left_ticks = action_to_ticks(actions[0]);
    right_ticks = action_to_ticks(actions[1]);
#endif
    action_q1000_for_log(actions, action_fixed, action_q1000);
    log_flags = log_flags_from_obs_command(obs_q1000, left_ticks, right_ticks);
    policy_end_ticks = read_clock_ticks();

#if LINE_FOLLOW_ENABLE_DRIVE
    drive_speed(left_ticks, right_ticks);
#endif
    drive_end_ticks = read_clock_ticks();
    loop_end_ticks = drive_end_ticks;
    policy_us = elapsed_us(qti_end_ticks, policy_end_ticks);
    drive_us = elapsed_us(policy_end_ticks, drive_end_ticks);
    body_us = elapsed_us(loop_start_ticks, loop_end_ticks);
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
#if LINE_FOLLOW_INFERENCE_MODE == LINE_FOLLOW_INFERENCE_FIXED
      print_action_fixed3(action_fixed[0]);
      print(" ");
      print_action_fixed3(action_fixed[1]);
#else
      print_float3(actions[0]);
      print(" ");
      print_float3(actions[1]);
#endif
      print(" %d %d %d %d %d %d %d %d %d\n",
          left_ticks, right_ticks, body_ms, period_us, body_us,
          qti_wait_us, policy_us, drive_us, qti_sample_us);
#if LINE_FOLLOW_QTI_MAP_MODE != LINE_FOLLOW_QTI_MAP_THREE_PIN
      print("P %d", loops);
      for(i = 0; i < QTI_PHYSICAL_COUNT; i++)
      {
        print_int_field(qti_last_physical_raw[i]);
      }
      print("\n");
#endif
    }

#if LINE_FOLLOW_SD_LOG
#if LINE_FOLLOW_SD_COG_LOG
    compact_trace_record(loops, elapsed_total_ms, period_us, raw,
        obs_q1000, action_q1000, left_ticks, right_ticks, log_flags);
#elif LINE_FOLLOW_SD_DEFERRED
    deferred_log_record(&deferred_log, loops, elapsed_total_ms, period_us,
        body_us, qti_wait_us, policy_us, drive_us, qti_sample_us, raw,
        obs_q1000, action_q1000, left_ticks, right_ticks, log_flags);
#else
#if LINE_FOLLOW_ENABLE_DRIVE
    drive_getTicks(&encoder_left_ticks, &encoder_right_ticks);
#endif
    if(sd_enabled && LINE_FOLLOW_SD_LOG_EVERY > 0
        && (loops % LINE_FOLLOW_SD_LOG_EVERY) == 0)
    {
      if(write_sd_row(sd_fp, loops, elapsed_total_ms, period_ms, body_ms,
          period_us, body_us, qti_wait_us, policy_us, drive_us, qti_sample_us,
          raw, obs_q1000, actions, action_fixed, left_ticks, right_ticks,
          encoder_left_ticks, encoder_right_ticks))
      {
        print("SD row write failed at loop=%d\n", loops);
        sd_failed = 1;
        sd_enabled = 0;
      }
      else
      {
        sd_records++;
        sd_total_records++;
        if(LINE_FOLLOW_SD_FLUSH_EVERY > 0
            && (sd_records % LINE_FOLLOW_SD_FLUSH_EVERY) == 0)
        {
          fflush(sd_fp);
        }
        if(LINE_FOLLOW_SD_CHUNK_RECORDS > 0
            && sd_records >= LINE_FOLLOW_SD_CHUNK_RECORDS)
        {
          close_sd_log(&sd_fp, sd_records, "chunk_records");
          sd_records = 0;
          sd_enabled = 0;
        }
        else if(LINE_FOLLOW_SD_MAX_RECORDS > 0
            && sd_total_records >= LINE_FOLLOW_SD_MAX_RECORDS)
        {
          close_sd_log(&sd_fp, sd_records, "max_records");
          sd_records = 0;
          sd_enabled = 0;
        }
      }
    }
#endif
#endif

#if LINE_FOLLOW_STATUS_LEDS
    if(((elapsed_total_ms / 250) & 1) == 0)
    {
      high(26);
      low(27);
    }
    else
    {
      low(26);
      high(27);
    }
#else
    set_output(26, obs_q1000[LEFT_SENSOR] >= QTI_THRESHOLD_Q1000);
    set_output(27, obs_q1000[RIGHT_SENSOR] >= QTI_THRESHOLD_Q1000);
#endif

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

  stop_qti_sampler_cog();

#if LINE_FOLLOW_SD_LOG
#if LINE_FOLLOW_SD_COG_LOG
  start_postrun_sd_cog();
  wait_postrun_sd_cog();
#elif LINE_FOLLOW_SD_DEFERRED
  deferred_log.final_elapsed_ms = run_start_ticks == 0
      ? 0
      : elapsed_ms(run_start_ticks, read_clock_ticks());
  sd_logger_state = LOGGER_IDLE;
  sd_logger_records = 0;
  sd_logger_cog = cogstart(sd_logger_main, 0,
      sd_logger_stack, sizeof(sd_logger_stack));
  if(sd_logger_cog >= 0)
  {
    unsigned int write_start_ticks = read_clock_ticks();
    sd_logger_state = LOGGER_REQUESTED;
    while(sd_logger_state == LOGGER_REQUESTED
        || sd_logger_state == LOGGER_WRITING)
    {
      high(26);
      low(27);
      pause(80);
      low(26);
      high(27);
      pause(80);
      if(LINE_FOLLOW_SD_WRITE_TIMEOUT_MS > 0
          && elapsed_ms(write_start_ticks, read_clock_ticks())
              >= LINE_FOLLOW_SD_WRITE_TIMEOUT_MS)
      {
        cogstop(sd_logger_cog);
        sd_logger_cog = -1;
        sd_logger_state = LOGGER_ERROR;
        print("SD logger cog timed out after %d ms\n",
            LINE_FOLLOW_SD_WRITE_TIMEOUT_MS);
        break;
      }
    }
    if(sd_logger_cog >= 0)
    {
      cogstop(sd_logger_cog);
      sd_logger_cog = -1;
    }
    print("SD logger cog state=%d records=%d\n",
        sd_logger_state, sd_logger_records);
  }
  else
  {
    print("SD logger cog start failed\n");
  }
#else
  if(sd_fp)
  {
    close_sd_log(&sd_fp, sd_records, "shutdown");
  }
#endif
#endif

  print("LINE_FOLLOW_MODEL_LIVE done\n");
  blink_done_forever();
}
#endif
