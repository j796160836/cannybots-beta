// David A. Mellis, 13 May 2010

#include <WProgram.h>
#undef ADC
#include <ADC.h>

#define NUM 512
int samples[NUM];

void setup()
{
  Serial.begin(115200);

  ADC.setPrescaleFactor(ADC.PRESCALE4);
  
  unsigned long start = millis(), end;
  for (int i = 0; i < NUM; i++) {
    samples[i] = analogRead(0);
  }
  end = millis();

  for (int i = 0; i < NUM; i++) {
    Serial.print(samples[i]);
    Serial.print(" ");
  }
  Serial.println();

//  Serial.println(end - start);
}

void loop()
{
}
