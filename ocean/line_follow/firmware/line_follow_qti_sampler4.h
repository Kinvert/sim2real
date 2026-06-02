#ifndef LINE_FOLLOW_QTI_SAMPLER4_H
#define LINE_FOLLOW_QTI_SAMPLER4_H

#include <stdint.h>

#define LINE_FOLLOW_QTI4_SENSOR_COUNT 4

typedef struct LineFollowQtiSampler4Mailbox {
  volatile int32_t control;
  volatile uint32_t seq;
  volatile int32_t raw[LINE_FOLLOW_QTI4_SENSOR_COUNT];
  volatile int32_t sample_us;
  volatile int32_t period_us;
  int32_t pin[LINE_FOLLOW_QTI4_SENSOR_COUNT];
  int32_t charge_ticks;
  int32_t timeout_ticks;
  int32_t sample_period_ticks;
  int32_t ticks_per_us;
} LineFollowQtiSampler4Mailbox;

#endif
