//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots Joypad
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0   -  14.08.2014  -  Inital Version 
//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <RFduinoGZLL.h>

#define XAXIS_PIN 2
#define YAXIS_PIN 3
#define BUTTON_PIN 4
#define MSG_LEN 4

#define GZLL_MAX_MSG_SIZE 32
char  gzllDebugBuf[GZLL_MAX_MSG_SIZE] = {0};
char* dbgMsg = NULL;


device_t role = DEVICE0;
char msg[MSG_LEN] = {0};  // 4 bytes  = 3 bytes data 1 byte NULL terminator

void setup()
{
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
   RFduinoGZLL.hostBaseAddress = 0x12ABCD00;
  RFduinoGZLL.begin(role);
}

void loop()
{
  bool buttonPressed = digitalRead(BUTTON_PIN)  & 0xFF;
  uint8_t xAxis = 255-(analogRead(XAXIS_PIN)/4) ;          // scale from 0..1023 to 0-255 so data fits in a byte
  uint8_t yAxis = 255-(analogRead(YAXIS_PIN)/4) ;
  
  snprintf(msg, MSG_LEN, "%c%c%c", xAxis, yAxis, buttonPressed );
  
  RFduinoGZLL.sendToHost((const char*)msg, MSG_LEN-1);      // don't bother sending the NULL byte at the end
  //Serial.write((uint8_t*)msg, MSG_LEN);
  /*
  Serial.print(xAxis);
  Serial.print("\t");  
  Serial.print(yAxis);
  Serial.print("\t");  
  Serial.println(buttonPressed);
  */
  delay(5);
  
  if (dbgMsg) {
      Serial.println(dbgMsg);
      dbgMsg=NULL;   
  }    
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  if (len > 0)
  {
    if (!dbgMsg) {
      memcpy( gzllDebugBuf, data, len);   
      dbgMsg=gzllDebugBuf;      
    }
  }
}

