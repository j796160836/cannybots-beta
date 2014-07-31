#include <avr/power.h>
#include <avr/sleep.h>

#include "Timer2.h"
#include "Sleep.h"

static void empty() {}

void sleep(unsigned int ms)
{
	long overflows = F_CPU / 256 * ms / 1000;
  uint8_t val;
  
  Timer2.setMode(Timer2.NORMAL);
  
#define SLEEP_CONFIG(cdiv, ps) \
do { \
  Timer2.setPrescaleFactor(Timer2.PRESCALE ## ps); \
  clock_prescale_set(clock_div_ ## cdiv); \
  val = ~(overflows * 256 / cdiv / ps); \
} while (0)

  if (overflows < 1) SLEEP_CONFIG(1, 1);
  else if (overflows < 8) SLEEP_CONFIG(1, 8);
  else if (overflows < 32) SLEEP_CONFIG(1, 32);
  else if (overflows < 64) SLEEP_CONFIG(1, 64);
  else if (overflows < 128) SLEEP_CONFIG(1, 128);
  else if (overflows < 1024) SLEEP_CONFIG(1, 1024);
  else if (overflows < 2048) SLEEP_CONFIG(2, 1024);
  else if (overflows < 4096) SLEEP_CONFIG(4, 1024);
  else if (overflows < 8192) SLEEP_CONFIG(8, 1024);
  else if (overflows < 16384) SLEEP_CONFIG(16, 1024);
  else if (overflows < 32768) SLEEP_CONFIG(32, 1024);
  else if (overflows < 65536) SLEEP_CONFIG(64, 1024);
  else if (overflows < 131072) SLEEP_CONFIG(128, 1024);
  else SLEEP_CONFIG(256, 1024);

	Timer2.write(0);
  Timer2.attachOverflowInterrupt(empty);
	Timer2.write(val);

  set_sleep_mode(SLEEP_MODE_EXT_STANDBY);
  sleep_mode();

	Timer2.detachOverflowInterrupt();
  clock_prescale_set(clock_div_1);
}
