//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots Radio Proxy
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version 1.0   -  14.08.2014  -  Inital Version
// Version 1.1   -  15.08.2014  -  Added BLE            (wayne@cannybots.com)
// Version 1.2   -  16.08.2014  -  Added Serial2Radio    (wayne@cannybots.com)
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

#define SERIAL2RADIO_MESSAGE_SIZE 20
#define RADIO2SERIAL_MESSAGE_SIZE 22

#define WATCHDOG_SETUP(seconds) NRF_WDT->CRV = 32768 * seconds; NRF_WDT->TASKS_START = 1;
#define WATCHDOG_RELOAD() NRF_WDT->RR[0] = WDT_RR_RR_Reload;


device_t role = HOST;
uint8_t buffer[RADIO2SERIAL_MESSAGE_SIZE] = {'>', '>'};
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
        RFduinoBLE_update_conn_interval(20, 20);
      }
      startGZLL   = !startGZLL;
    }
  }

  if (send) {
    Serial.write(buffer, RADIO2SERIAL_MESSAGE_SIZE);
    //Serial.flush();
    send = false;
  }

  processSerial2Radio();
}


void copyData(char *data, int len) {  
  // has to be at 6 least bytes:  [ID(1)][VARNAME(5)]
  if (len >= 6) {
    memcpy(buffer + 2, data, min(20,len));
    send = true;
  }
}


void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  gzllConnected = true;
  gzllConnectionTimeout = timeNow + GZLL_CONNECTION_TIMEOUT;
  copyData(data, len);
}


void RFduinoBLE_onConnect() {
  bleConnected = true;
}

void RFduinoBLE_onDisconnect() {
  bleConnected = false;
}

void RFduinoBLE_onReceive(char *data, int len) {
  copyData(data, len);
}

//TODO: void RFduinoBLE_onRSSI(int rssi) {
//}

// Serial Input
// We're expecting messages of 20 bytes in the form:  
// Where;
// >> = start marker, as-is
// 20 bytes of data to forward
void processSerial2Radio() {
  if (!bleConnected && !gzllConnected) 
    return;
  char msg[SERIAL2RADIO_MESSAGE_SIZE]={0};
  static int c = 0, lastChar = 0;
  while (Serial.available() >= SERIAL2RADIO_MESSAGE_SIZE+2) {
    lastChar = c;
    c = Serial.read();
    if ( ('>' == c) && ('>' == lastChar) ) {
      for (int i = 0; i < SERIAL2RADIO_MESSAGE_SIZE; i++) {
        msg[i] = Serial.read();
      }
      if (bleConnected) {
        RFduinoBLE.send((const char*)msg, SERIAL2RADIO_MESSAGE_SIZE);
      } else if (gzllConnected) {
        // Drop message for now, the Gazell based joypad is 'dumb', it has no outputs.
        // TODO: GZLL can only 'piggyback' on DEVICE to (this) HOST, so we would need to store and forward
      } else {
        // Drop, no point caching data.
      }

      lastChar = c = 0;
    }
  }
}
