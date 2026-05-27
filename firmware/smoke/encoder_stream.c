/*
  ActivityBot encoder stream for a host-side dashboard.

  This reads the original ActivityBot encoder pins directly:
  P14 = left encoder signal, P15 = right encoder signal.
  It does not command servos and does not write EEPROM.
*/

#include "simpletools.h"

int main(void)
{
  int left_raw = input(14);
  int right_raw = input(15);
  int left_edges = 0;
  int right_edges = 0;
  int loops = 0;

  low(26);
  low(27);

  print("\nENCODER_STREAM v1\n");
  print("E %d %d %d %d\n", left_edges, right_edges, left_raw, right_raw);

  while(1)
  {
    int next_left = input(14);
    int next_right = input(15);

    if(next_left != left_raw)
    {
      left_raw = next_left;
      left_edges++;
      set_output(26, left_raw);
    }

    if(next_right != right_raw)
    {
      right_raw = next_right;
      right_edges++;
      set_output(27, right_raw);
    }

    loops++;
    if(loops >= 10)
    {
      loops = 0;
      print("E %d %d %d %d\n", left_edges, right_edges, left_raw, right_raw);
    }

    pause(5);
  }
}
