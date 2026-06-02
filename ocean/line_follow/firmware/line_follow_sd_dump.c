/*
  RAM-only microSD log extractor.

  This does not command motors and does not write EEPROM. It mounts the
  ActivityBoard microSD card, reads the line-follow binary log, and streams it
  over the USB serial terminal as hex-framed lines that the host can decode.
*/

#include "simpletools.h"
#include <stdio.h>

#ifndef LINE_FOLLOW_SD_FILE
#define LINE_FOLLOW_SD_FILE "lf_log.bin"
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

#define DUMP_CHUNK_BYTES 32

static const char hex_digits[] = "0123456789abcdef";

static void encode_hex(const unsigned char* bytes, int count, char* out)
{
  int i;
  for(i = 0; i < count; i++)
  {
    unsigned char value = bytes[i];
    out[2 * i] = hex_digits[value >> 4];
    out[2 * i + 1] = hex_digits[value & 15];
  }
  out[2 * count] = 0;
}

int main(void)
{
  FILE* fp;
  unsigned char bytes[DUMP_CHUNK_BYTES];
  char hex[DUMP_CHUNK_BYTES * 2 + 1];
  int err;
  int total = 0;

  low(26);
  low(27);

  print("\nLF_SD_DUMP v1\n");
  print("file=%s pins do=%d clk=%d di=%d cs=%d\n",
      LINE_FOLLOW_SD_FILE, LINE_FOLLOW_SD_DO_PIN, LINE_FOLLOW_SD_CLK_PIN,
      LINE_FOLLOW_SD_DI_PIN, LINE_FOLLOW_SD_CS_PIN);

  err = sd_mount(LINE_FOLLOW_SD_DO_PIN, LINE_FOLLOW_SD_CLK_PIN,
      LINE_FOLLOW_SD_DI_PIN, LINE_FOLLOW_SD_CS_PIN);
  if(err)
  {
    print("LF_SD_DUMP_ERROR mount err=%d\n", err);
    high(26);
    while(1) pause(1000);
  }

  fp = fopen(LINE_FOLLOW_SD_FILE, "r");
  if(!fp)
  {
    print("LF_SD_DUMP_ERROR open file=%s\n", LINE_FOLLOW_SD_FILE);
    high(27);
    while(1) pause(1000);
  }

  print("LF_SD_DUMP_BEGIN chunk=%d\n", DUMP_CHUNK_BYTES);
  while(1)
  {
    int count = (int)fread(bytes, 1, DUMP_CHUNK_BYTES, fp);
    if(count <= 0)
    {
      break;
    }
    encode_hex(bytes, count, hex);
    print("H %s\n", hex);
    total += count;
  }
  fclose(fp);

  print("LF_SD_DUMP_END bytes=%d\n", total);
  while(1)
  {
    pause(1000);
  }
}
