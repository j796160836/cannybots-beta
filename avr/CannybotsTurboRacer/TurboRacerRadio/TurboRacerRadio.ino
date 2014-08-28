//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots Radio Proxy
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version 1.0   -  14.08.2014  -  Inital Version 
//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <RFduinoGZLL.h>

#define TX_PIN 5
#define RX_PIN 6
#define BUF_LEN 6

device_t role = HOST;
uint8_t buffer[BUF_LEN] = {'>', '>'};
volatile bool send = false;

void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);        // UART Baud is limited to 9600 when the BLE stack is on.
  RFduinoGZLL.hostBaseAddress = 0x12ABCD12;
  RFduinoGZLL.begin(role);
}

void loop() {
  if (send) {
    Serial.write(buffer, BUF_LEN);
    Serial.flush();
    send=false;
  }
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  if (len >= 3) {
    buffer[2] = device & 0xFF;  // device_t is an enum 0-7 for DEVICE[0..7] and 8 = HOST, defined in libRFduinoGZLL.h
    memcpy(buffer+3,data,3);
    send=true;
  }
}
