/*
  Brief ActivityBot drive-speed probe.

  Loads to RAM, commands fixed left/right ticks per second for a short duration,
  prints encoder ticks, then stops. Use this to verify whether equal wheel
  commands actually drive straight on the current robot.
*/

#include "simpletools.h"
#include "abdrive.h"

#ifndef DRIVE_LEFT_TICKS
#define DRIVE_LEFT_TICKS 2
#endif

#ifndef DRIVE_RIGHT_TICKS
#define DRIVE_RIGHT_TICKS 2
#endif

#ifndef DRIVE_RUN_MS
#define DRIVE_RUN_MS 3000
#endif

#ifndef DRIVE_PRINT_MS
#define DRIVE_PRINT_MS 250
#endif

int main(void)
{
  int elapsed = 0;
  int left_ticks = 0;
  int right_ticks = 0;

  low(26);
  low(27);

  print("\nActivityBot drive ticks probe\n");
  print("command left=%d right=%d run_ms=%d print_ms=%d\n",
      DRIVE_LEFT_TICKS, DRIVE_RIGHT_TICKS, DRIVE_RUN_MS, DRIVE_PRINT_MS);
  print("This loads to RAM, commands motors briefly, then stops.\n");
  print("D elapsed_ms left_ticks right_ticks\n");

  drive_setRampStep(4);
  drive_speed(0, 0);
  pause(250);
  drive_getTicks(&left_ticks, &right_ticks);
  print("D %d %d %d\n", elapsed, left_ticks, right_ticks);

  drive_speed(DRIVE_LEFT_TICKS, DRIVE_RIGHT_TICKS);

  while(elapsed < DRIVE_RUN_MS)
  {
    pause(DRIVE_PRINT_MS);
    elapsed += DRIVE_PRINT_MS;
    drive_getTicks(&left_ticks, &right_ticks);
    print("D %d %d %d\n", elapsed, left_ticks, right_ticks);
  }

  drive_speed(0, 0);
  pause(250);
  drive_getTicks(&left_ticks, &right_ticks);
  print("D stopped %d %d %d\n", elapsed, left_ticks, right_ticks);
  drive_close();
  print("ActivityBot drive ticks probe done\n");

  while(1)
  {
    pause(1000);
  }
}
