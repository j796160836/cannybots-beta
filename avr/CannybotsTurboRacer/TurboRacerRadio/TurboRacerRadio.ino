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
/////////////////////////////////////////////////////////
// Config

#define TOGGLE_MILLIS 1500
#define GZLL_CONNECTION_TIMEOUT 2000

// BLE Data
#define BLE_UUID                 "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_ADVERTISEMENT_DATA_MAX 16

// TX Power
// x one of;  (low) -20, -16, -12, -8, -4, 0, +4 (high & default)
// RFduino default is +4
#define BLE_TX_POWER_LEVEL  0
#define GZLL_TX_POWER_LEVEL 0

#if 1
#define TX_PIN 5
#define RX_PIN 6
#else
#define TX_PIN 1
#define RX_PIN 0
#endif

#define BUF_LEN 6

#define WATCHDOG_SETUP(seconds) NRF_WDT->CRV = 32768 * seconds; NRF_WDT->TASKS_START = 1;
#define WATCHDOG_RELOAD() NRF_WDT->RR[0] = WDT_RR_RR_Reload;


device_t role = HOST;
uint8_t buffer[BUF_LEN] = {'>', '>'};
volatile bool send = false;
volatile bool startGZLL = true;
volatile bool bleConnected = false;
volatile bool gzllConnected = false;
unsigned long nextRadioToggleTime = millis();
volatile unsigned long timeNow = millis();
volatile unsigned long gzllConnectionTimeout = 0;
char bleName[BLE_ADVERTISEMENT_DATA_MAX] = {0};

void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);        // UART Baud is limited to 9600 when the BLE stack is on.

  snprintf(bleName, BLE_ADVERTISEMENT_DATA_MAX, "CB_%x%x", getDeviceIdHigh, getDeviceIdLow());

  RFduinoBLE.txPowerLevel      = BLE_TX_POWER_LEVEL;
  RFduinoGZLL.txPowerLevel     = GZLL_TX_POWER_LEVEL;
  RFduinoBLE.customUUID        = BLE_UUID;
  RFduinoBLE.deviceName        = bleName;
  RFduinoBLE.advertisementData = bleName;
  //RFduinoBLE.advertisementInterval(millis);
  
  WATCHDOG_SETUP(7);
}

void loop() {
  timeNow   = millis();
  WATCHDOG_RELOAD();
  
   if ( gzllConnected && (timeNow  > gzllConnectionTimeout)) {
      gzllConnected = false;
   }

  if (!bleConnected && !gzllConnected) {
    if (millis() > nextRadioToggleTime) {
      //Serial.println("toggle");
      nextRadioToggleTime = millis() + TOGGLE_MILLIS;

      if (startGZLL) {
        while (RFduinoBLE.radioActive);
        RFduinoBLE.end();
        delay(50);
        RFduinoGZLL.begin(HOST);
      } else  {
        RFduinoGZLL.end();
        delay(50);
        RFduinoBLE.begin();
      }
      startGZLL   = !startGZLL;
    }
  }

  if (send) {
    Serial.write(buffer, BUF_LEN);
    //Serial.flush();
    send = false;
  }
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  gzllConnected = true;
  gzllConnectionTimeout = timeNow + GZLL_CONNECTION_TIMEOUT;
  
  if (len >= 3) {
    buffer[2] = device & 0xFF;  // device_t is an enum 0-7 for DEVICE[0..7] and 8 = HOST, defined in libRFduinoGZLL.h
    memcpy(buffer + 3, data, 3);
    send = true;
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
    buffer[2] = data[0]; // phone send 4 bytes:  ID, X, Y, B
    memcpy(buffer + 3, data+1, 3);
    send = true;
  }
}

