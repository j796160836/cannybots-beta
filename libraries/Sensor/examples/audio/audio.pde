#include <WProgram.h>
#undef ADC
#include <ADC.h>
#include <Sensor.h>
#include <Sleep.h>
#include <Timer1.h>
#include <Timer2.h>

#define SAMPLES 1000

volatile int i = 0;
volatile byte samples[SAMPLES];
volatile byte transmit;

void setup()
{
  Serial.begin(115200);
  analogReference(INTERNAL);
  autoSample(0, 44100, sample);
}

void loop()
{
  if (transmit) {
    transmit = false;
    for (i = 0; i < SAMPLES; i++) {
      Serial.print(samples[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
    i = 0;
    autoSample(0, 44100, sample);
  }
}

void sample(int val)
{
  samples[i++] = val >> 2;
  if (i == SAMPLES) {
    noAutoSample();
    transmit = true;
  }
}
