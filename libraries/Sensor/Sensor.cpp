#include <WProgram.h>

#include "ADC.h"
#include "Sensor.h"
#include "Timer1.h"

extern uint8_t analog_reference;

static void (*isr)(int) = 0;
static void sample()
{
	if (isr) isr(ADC.read());
}

void autoSample(int channel, long rate, void (*f)(int))
{
	long cycles = F_CPU / rate;
  
  isr = f;
  
  Timer1.setMode(Timer1.CTC);
  Timer1.write(0);
  Timer1.writeCompareB(0);

#define TIMER_CONFIG(ps) \
do { \
  Timer1.setPrescaleFactor(Timer1.PRESCALE ## ps); \
  Timer1.writeCompareA(cycles / ps); \
} while (0)

	if (cycles < 65536) TIMER_CONFIG(1);
  else if (cycles < 524288) TIMER_CONFIG(8);
  else if (cycles < 4194304) TIMER_CONFIG(64);
  else TIMER_CONFIG(256);

  ADC.setPrescaleFactor(ADC.PRESCALE16);
  ADC.setReference(analog_reference);
  ADC.setMUX(channel);
  ADC.attachInterrupt(sample);
  ADC.setAutoTriggerSource(ADC.TIMER1_COMPAREB);
  ADC.autoTrigger();
}

void noAutoSample()
{
	ADC.noAutoTrigger();
  ADC.detachInterrupt();
}

int vcc()
{
	ADC.setReference(DEFAULT);
  ADC.setMUX(ADC.MUX1V1);
  ADC.begin();
  while (ADC.isRunning());
  return 1024L * 1100L / ADC.read();
}
