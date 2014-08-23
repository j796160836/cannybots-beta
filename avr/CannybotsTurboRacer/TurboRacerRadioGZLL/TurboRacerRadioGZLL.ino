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
#define RADIO2SERIAL_MESSAGE_SIZE 22

device_t role = HOST;
uint8_t buffer[RADIO2SERIAL_MESSAGE_SIZE] = {'>', '>'};
volatile bool send = false;


void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);        // UART Baud is limited to 9600 when the BLE stack is on.
  //RFduinoGZLL.hostBaseAddress=0x0D0A0704;  // NRF default
  //RFduinoGZLL.deviceBaseAddress0x0E0B0805; // NRF default
  RFduinoGZLL.hostBaseAddress = 0x91827364;
  RFduinoGZLL.begin(role);
}


void loop() {
  if (send) {
    Serial.write(buffer, RADIO2SERIAL_MESSAGE_SIZE);
    //Serial.flush();
    send = false;
  }
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  if (len >= 3) {
    buffer[2] = device & 0xFF;  // device_t is an enum 0-7 for DEVICE[0..7] and 8 = HOST, defined in libRFduinoGZLL.h
    memcpy(buffer + 3, data, 3);
    send = true;
  }
}

