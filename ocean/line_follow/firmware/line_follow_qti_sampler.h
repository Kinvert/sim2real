#ifndef LINE_FOLLOW_QTI_SAMPLER_H
#define LINE_FOLLOW_QTI_SAMPLER_H

#include <stdint.h>

#define LINE_FOLLOW_QTI_SENSOR_COUNT 3

typedef struct LineFollowQtiSamplerMailbox {
  volatile int32_t control;
  volatile uint32_t seq;
  volatile int32_t raw[LINE_FOLLOW_QTI_SENSOR_COUNT];
  volatile int32_t sample_us;
  volatile int32_t period_us;
  int32_t pin[LINE_FOLLOW_QTI_SENSOR_COUNT];
  int32_t charge_ticks;
  int32_t timeout_ticks;
  int32_t sample_period_ticks;
  int32_t ticks_per_us;
} LineFollowQtiSamplerMailbox;

#endif
