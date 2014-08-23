//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots Radio Proxy (BLE only)
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version 1.0   -  22.08.2014  -  Inital Version
//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <RFduinoBLE.h>

#if 1
#define TX_PIN 5
#define RX_PIN 6
#else
#define TX_PIN 1
#define RX_PIN 0
#endif
#define RADIO2SERIAL_MESSAGE_SIZE   6
#define SERIAL2RADIO_MESSAGE_SIZE   20

#define BLE_UUID                   "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_ADVERTISEMENT_DATA_MAX 16
#define BLE_TX_POWER_LEVEL         0

char bleName[BLE_ADVERTISEMENT_DATA_MAX] = {0};
volatile bool bleConnected = false;

uint8_t buffer[RADIO2SERIAL_MESSAGE_SIZE] = {'$', '$'};
volatile bool send = false;


void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);        // UART Baud is limited to 9600 when the BLE stack is on.
  snprintf(bleName, BLE_ADVERTISEMENT_DATA_MAX, "CB_%x%x", getDeviceIdHigh, getDeviceIdLow());

  RFduinoBLE.txPowerLevel      = BLE_TX_POWER_LEVEL;
  RFduinoBLE.customUUID        = BLE_UUID;
  RFduinoBLE.deviceName        = bleName;
  RFduinoBLE.advertisementData = bleName;
  RFduinoBLE.begin();
  //RFduinoBLE_update_conn_interval(20, 20);
}


void loop() {
  if (send) {
    Serial.write(buffer, RADIO2SERIAL_MESSAGE_SIZE);
    //Serial.flush();
    send = false;
  }
}

void RFduinoBLE_onConnect() {
  bleConnected = true;
}

void RFduinoBLE_onDisconnect() {
  bleConnected = false;
}

void RFduinoBLE_onReceive(char *data, int len) {
  if (len >=6) {
    copyData(data, len);
    send = true;
  }
}



void copyData(char *data, int len) {
  memcpy(buffer + 2, data, len);
  send = true;
}

// Serial Input
// We're expecting messages of 20 bytes in the form:
// Where;
// >> = start marker, as-is
// 20 bytes of data to forward
void processSerial2Radio() {
  char msg[SERIAL2RADIO_MESSAGE_SIZE] = {0};
  static int c = 0, lastChar = 0;
  while (Serial.available() >= SERIAL2RADIO_MESSAGE_SIZE + 2) {
    lastChar = c;
    c = Serial.read();
    if ( ('>' == c) && ('>' == lastChar) ) {
      for (int i = 0; i < SERIAL2RADIO_MESSAGE_SIZE; i++) {
        msg[i] = Serial.read();
      }
      if (bleConnected) {
        RFduinoBLE.send((const char*)msg, SERIAL2RADIO_MESSAGE_SIZE);
      }
      lastChar = c = 0;
    }
  }
}


