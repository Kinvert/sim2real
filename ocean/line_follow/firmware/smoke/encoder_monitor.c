/*
  ActivityBot encoder monitor.

  This is a read-only wiring test for the original external encoder pins:
  P14 = left encoder signal, P15 = right encoder signal.
*/

#include "simpletools.h"

int main(void)
{
  int left_raw = input(14);
  int right_raw = input(15);
  int left_edges = 0;
  int right_edges = 0;

  low(26);
  low(27);

  print("\nActivityBot encoder monitor\n");
  print("Spin wheels by hand. P14=left encoder, P15=right encoder.\n");
  print("left_raw right_raw left_edges right_edges\n");

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

    print("%d %d %d %d\n", left_raw, right_raw, left_edges, right_edges);
    pause(100);
  }
}
