//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots Radio Proxy
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version 1.0   -  14.08.2014  -  Inital Version 
// Version 1.1   -  15.08.2014  -  Add BLE         (wayne@cannybots.com)
//
//////////////////////////////////////////////////////////////////////////////////////////////////


#include <RFduinoGZLL.h>
#include <RFduinoBLE.h>

#define TX_PIN 5
#define RX_PIN 6
#define BUF_LEN 6

device_t role = HOST;
uint8_t buffer[BUF_LEN] = {'>', '>'};
volatile bool send = false;
volatile bool startGZLL = true;
volatile bool startBLE  = false;
volatile bool bleConnected = false;
volatile bool gzllConnected = false;

void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);        // UART Baud is limited to 9600 when the BLE stack is on.
}

void loop() {
  
  if (startGZLL) {
    startGZLL=false;
    while (RFduinoBLE.radioActive);
    RFduinoBLE.end();
    delay(50);
    RFduinoGZLL.begin(HOST);  
  }
  if (startBLE) {
    startBLE=false;
    RFduinoGZLL.end();  
    delay(50);
    RFduinoBLE.begin();
  }

  
  if (send) {
    Serial.write(buffer, BUF_LEN);
    Serial.flush();
    send=false;
  }
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  gzllConnected=true;
  if (len >= 3) {
    buffer[2] = device & 0xFF;  // device_t is an enum 0-7 for DEVICE[0..7] and 8 = HOST, defined in libRFduinoGZLL.h
    memcpy(buffer+3,data,3);
    send=true;
  }
}


void RFduinoBLE_onConnect() {
  bleConnected = true;
}

void RFduinoBLE_onDisconnect() {
  bleConnected = false;
}

void RFduinoBLE_onReceive(char *data, int len) {
  if (len >= 3) {
    buffer[2] = data[0]; // phoen send 4 bytes:  ID, X, Y, B 
    memcpy(buffer+3,data,3);
    send=true;
  }  
}
