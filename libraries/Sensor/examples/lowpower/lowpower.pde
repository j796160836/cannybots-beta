#include <ADC.h>
#include <Sleep.h>
#include <Timer1.h>
#include <Timer2.h>

void setup()
{
  Serial.begin(115200);
}

void loop()
{
//  delay(1000);
  sleep(1000);
  Serial.println(analogRead(0));
}
