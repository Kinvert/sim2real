/*
  Line-follow Propeller C sanity harness.

  Default build is telemetry-only: it reads four QTI RC-time values, normalizes
  them to the sim polarity, runs a small policy-shaped stub, and prints the
  resulting wheel commands. Motor output is compiled in only when
  LINE_FOLLOW_ENABLE_DRIVE is set to 1.
*/

#include "simpletools.h"

#ifndef LINE_FOLLOW_ENABLE_DRIVE
#define LINE_FOLLOW_ENABLE_DRIVE 0
#endif

#ifndef LINE_FOLLOW_FAKE_OBS
#define LINE_FOLLOW_FAKE_OBS 0
#endif

#if LINE_FOLLOW_ENABLE_DRIVE
#include "abdrive.h"
#endif

#ifndef QTI_OUTER_LEFT_PIN
#define QTI_OUTER_LEFT_PIN 7
#endif

#ifndef QTI_INNER_LEFT_PIN
#define QTI_INNER_LEFT_PIN 6
#endif

#ifndef QTI_INNER_RIGHT_PIN
#define QTI_INNER_RIGHT_PIN 5
#endif

#ifndef QTI_OUTER_RIGHT_PIN
#define QTI_OUTER_RIGHT_PIN 4
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

#ifndef LINE_FOLLOW_BASE_TICKS
#define LINE_FOLLOW_BASE_TICKS 20
#endif

#ifndef LINE_FOLLOW_TURN_TICKS
#define LINE_FOLLOW_TURN_TICKS 18
#endif

#ifndef LINE_FOLLOW_SEARCH_TICKS
#define LINE_FOLLOW_SEARCH_TICKS 12
#endif

#ifndef LINE_FOLLOW_MAX_TICKS
#define LINE_FOLLOW_MAX_TICKS 40
#endif

#ifndef LINE_FOLLOW_LOOP_MS
#define LINE_FOLLOW_LOOP_MS 25
#endif

#ifndef LINE_FOLLOW_MAX_LOOPS
#define LINE_FOLLOW_MAX_LOOPS 0
#endif

#define OBS_COUNT 4

typedef struct {
  const char* name;
  int obs[OBS_COUNT];
} FakeCase;

static int qti_pins[OBS_COUNT] = {
  QTI_OUTER_LEFT_PIN,
  QTI_INNER_LEFT_PIN,
  QTI_INNER_RIGHT_PIN,
  QTI_OUTER_RIGHT_PIN
};

static int clamp_int(int value, int lo, int hi)
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

static void read_qti(int raw[OBS_COUNT], int obs[OBS_COUNT])
{
  int i;
  for(i = 0; i < OBS_COUNT; i++)
  {
    raw[i] = read_qti_pin(qti_pins[i]);
    obs[i] = normalize_q1000(raw[i]);
  }
}

static void policy_stub(const int obs[OBS_COUNT], int* left_ticks, int* right_ticks)
{
  int left_dark = obs[0] + obs[1];
  int right_dark = obs[2] + obs[3];
  int strongest = 0;
  int correction;

  int i;
  for(i = 0; i < OBS_COUNT; i++)
  {
    if(obs[i] > strongest)
    {
      strongest = obs[i];
    }
  }

  if(strongest < QTI_THRESHOLD_Q1000)
  {
    *left_ticks = LINE_FOLLOW_SEARCH_TICKS;
    *right_ticks = -LINE_FOLLOW_SEARCH_TICKS;
    return;
  }

  correction = ((left_dark - right_dark) * LINE_FOLLOW_TURN_TICKS) / 1000;
  *left_ticks = clamp_int(LINE_FOLLOW_BASE_TICKS - correction,
      -LINE_FOLLOW_MAX_TICKS, LINE_FOLLOW_MAX_TICKS);
  *right_ticks = clamp_int(LINE_FOLLOW_BASE_TICKS + correction,
      -LINE_FOLLOW_MAX_TICKS, LINE_FOLLOW_MAX_TICKS);
}

static void print_q1000(int value)
{
  int clipped = clamp_int(value, 0, 1000);
  print("%d.%03d", clipped / 1000, clipped % 1000);
}

static void run_fake_obs_cases(void)
{
  int i;
  int left_ticks = 0;
  int right_ticks = 0;
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

  print("FAKE_OBS mode: no QTI reads, no drive output\n");
  print("F name obs0 obs1 obs2 obs3 left right\n");
  for(i = 0; i < num_cases; i++)
  {
    policy_stub(cases[i].obs, &left_ticks, &right_ticks);
    print("F %s ", cases[i].name);
    print_q1000(cases[i].obs[0]);
    print(" ");
    print_q1000(cases[i].obs[1]);
    print(" ");
    print_q1000(cases[i].obs[2]);
    print(" ");
    print_q1000(cases[i].obs[3]);
    print(" %d %d\n", left_ticks, right_ticks);
  }
}

int main(void)
{
  int raw[OBS_COUNT];
  int obs[OBS_COUNT];
  int left_ticks = 0;
  int right_ticks = 0;
  int loops = 0;

  low(26);
  low(27);

  print("\nLINE_FOLLOW_SANITY v1 drive=%d\n", LINE_FOLLOW_ENABLE_DRIVE);
  print("pins outer_left=%d inner_left=%d inner_right=%d outer_right=%d\n",
      QTI_OUTER_LEFT_PIN, QTI_INNER_LEFT_PIN, QTI_INNER_RIGHT_PIN, QTI_OUTER_RIGHT_PIN);
  print("cal white=%d black=%d threshold_q1000=%d loop_ms=%d\n",
      QTI_WHITE_TIME, QTI_BLACK_TIME, QTI_THRESHOLD_Q1000, LINE_FOLLOW_LOOP_MS);

#if LINE_FOLLOW_FAKE_OBS
  run_fake_obs_cases();
  print("LINE_FOLLOW_SANITY done\n");
  while(1) pause(1000);
#endif

  print("S raw0 raw1 raw2 raw3 obs0 obs1 obs2 obs3 left right\n");

#if LINE_FOLLOW_ENABLE_DRIVE
  drive_setRampStep(4);
  drive_speed(0, 0);
  pause(500);
#endif

  while(LINE_FOLLOW_MAX_LOOPS == 0 || loops < LINE_FOLLOW_MAX_LOOPS)
  {
    read_qti(raw, obs);
    policy_stub(obs, &left_ticks, &right_ticks);

    print("S %d %d %d %d %d %d %d %d %d %d\n",
        raw[0], raw[1], raw[2], raw[3],
        obs[0], obs[1], obs[2], obs[3],
        left_ticks, right_ticks);

#if LINE_FOLLOW_ENABLE_DRIVE
    drive_speed(left_ticks, right_ticks);
#endif

    set_output(26, obs[1] >= QTI_THRESHOLD_Q1000);
    set_output(27, obs[2] >= QTI_THRESHOLD_Q1000);

    loops++;
    pause(LINE_FOLLOW_LOOP_MS);
  }

#if LINE_FOLLOW_ENABLE_DRIVE
  drive_speed(0, 0);
  pause(250);
  drive_close();
#endif

  print("LINE_FOLLOW_SANITY done\n");
  while(1) pause(1000);
}
