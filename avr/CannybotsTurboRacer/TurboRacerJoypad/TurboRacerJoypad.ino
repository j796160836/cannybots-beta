//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots Joypad
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0   -  14.08.2014  -  Inital Version 
// Version:   1.1   -  16.08.2014  -  Use key/type/value pairs
//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <RFduinoGZLL.h>

#define XAXIS_PIN 2
#define YAXIS_PIN 3
#define BUTTON_PIN 4
#define MSG_LEN 21

device_t role = DEVICE0;

char msg[MSG_LEN] = {0};  

void setup()
{
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  RFduinoGZLL.begin(role);
}

void loop()
{
  bool buttonPressed = digitalRead(BUTTON_PIN)  & 0xFF;
  int16_t xAxis = map(analogRead(XAXIS_PIN), 0,1023, 255, -255) ;          // scale from 0..1023 to 0-255 so data fits in a byte
  int16_t yAxis = map(analogRead(YAXIS_PIN), 0,1023, 255, -255) ;
  
  snprintf(msg, sizeof(msg), "%c%5.5s%c% .10d  ", role, "JOY_X", 'd', xAxis);
  //RFduinoGZLL.sendToHost((const char*)msg, MSG_LEN-1);      // don't bother sending the NULL byte at the end
  snprintf(msg, sizeof(msg), "%c%5.5s%c% .10d  ", role, "JOY_Y", 'd', yAxis);
  RFduinoGZLL.sendToHost((const char*)msg, MSG_LEN-1);      // don't bother sending the NULL byte at the end
  snprintf(msg, sizeof(msg), "%c%5.5s%c% .10d  ", role, "JOYB1", 'd', buttonPressed);
  //RFduinoGZLL.sendToHost((const char*)msg, MSG_LEN-1);      // don't bother sending the NULL byte at the end

  Serial.write((uint8_t*)msg, MSG_LEN);
  Serial.println();

  delay(50);
}


