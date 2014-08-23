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
// Version:   1.2   -  17.08.2014  -  Use binary encoding for message variables
//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <RFduinoGZLL.h>

#define JOYPAD_ID    1
#define XAXIS_PIN    2
#define YAXIS_PIN    3
#define BUTTON_PIN   4
#define MSG_LEN     21

device_t role = DEVICE1;

char msg[MSG_LEN] = {0};  


// Sleep support
#define AXIS_DEADZONE  5
#define SLEEP_TIMEOUT  3000
char lastMsg[MSG_LEN] = {0};
unsigned long lastTouchTime = millis();
unsigned long nextSleepTime = lastTouchTime + SLEEP_TIMEOUT;

void setup()
{
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  RFduinoGZLL.begin(role);
  
    
  pinMode(BUTTON_PIN,INPUT_PULLDOWN);
  RFduino_pinWake(BUTTON_PIN,HIGH);
}

void loop()
{
  bool buttonPressed = digitalRead(BUTTON_PIN)  & 0xFF;
  int16_t xAxis = map(analogRead(XAXIS_PIN), 0,1023, 255, -255) ;          
  int16_t yAxis = map(analogRead(YAXIS_PIN), 0,1023, 255, -255) ;
  
  xAxis = abs(xAxis)<AXIS_DEADZONE?0:xAxis;
  yAxis = abs(yAxis)<AXIS_DEADZONE?0:yAxis;
  
  snprintf(msg, sizeof(msg), "%c%5.5s%c%c%c%c%c%c", JOYPAD_ID, "JOY01", highByte(xAxis), lowByte(xAxis), highByte(yAxis), lowByte(yAxis), highByte(buttonPressed), lowByte(buttonPressed));
  RFduinoGZLL.sendToHost((const char*)msg, 12);      
  Serial.write((uint8_t*)msg, 12);
  Serial.println();
  delay(50);
  
  // Do sleep detection based on message changes
  if (strncmp(msg, lastMsg, MSG_LEN) != 0 ) {
    lastTouchTime = millis();
  }
  
  if ( millis()-lastTouchTime > SLEEP_TIMEOUT) {
      RFduinoBLE_ULPDelay(INFINITE);
  }
  strncpy(lastMsg, msg, MSG_LEN);  

}


