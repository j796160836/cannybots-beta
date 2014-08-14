#include <RFduinoGZLL.h>

#define XAXIS_PIN 2
#define YAXIS_PIN 3
#define BUTTON_PIN 4
#define MSG_LEN 6

device_t role = DEVICE0;
char msg[MSG_LEN] = {0};  // 5 bytes + NULL terminator for message

void setup()
{
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  RFduinoGZLL.begin(role);
}

void loop()
{
  bool buttonPressed = digitalRead(BUTTON_PIN)  & 0xFF;
  uint8_t xAxis = 255-(analogRead(XAXIS_PIN)>>2) ;
  uint8_t yAxis = 255-(analogRead(YAXIS_PIN)>>2) ;
  snprintf(msg, MSG_LEN, ">>%c%c%c", xAxis, yAxis, buttonPressed );
  RFduinoGZLL.sendToHost((const char*)msg, MSG_LEN-1);
  //Serial.write((uint8_t*)msg, MSG_LEN-1);
  delay(5);
}


