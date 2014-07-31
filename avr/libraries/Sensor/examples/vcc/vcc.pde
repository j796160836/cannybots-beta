#include <WProgram.h>
#undef ADC
#include <ADC.h>
#include <Sensor.h>
#include <Sleep.h>
#include <Timer1.h>
#include <Timer2.h>

void setup()
{
  Serial.begin(19200);
}

void loop()
{
  Serial.println(vcc());
  delay(1000);
}
