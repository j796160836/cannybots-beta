#include <RFduinoGZLL.h>

#define XAXIS_PIN 2
#define YAXIS_PIN 3
#define BUTTON_PIN 4
#define MSG_LEN 4

device_t role = DEVICE0;
char msg[MSG_LEN] = {0};  // 4 bytes  = 3 bytes data 1 byte NULL terminator

void setup()
{
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  RFduinoGZLL.begin(role);
}

void loop()
{
  bool buttonPressed = digitalRead(BUTTON_PIN)  & 0xFF;
  uint8_t xAxis = 255-(analogRead(XAXIS_PIN)>>2) ;          // scale from 0..1023 to 0-255 so data fits in a byte
  uint8_t yAxis = 255-(analogRead(YAXIS_PIN)>>2) ;
  
  snprintf(msg, MSG_LEN, "%c%c%c", xAxis, yAxis, buttonPressed );
  
  RFduinoGZLL.sendToHost((const char*)msg, MSG_LEN-1);      // don't bother sending the NULL byte at the end
  Serial.write((uint8_t*)msg, MSG_LEN);
  delay(5);
}


