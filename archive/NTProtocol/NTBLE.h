#ifndef NTBLE_H
#define NTBLE_H
#ifdef ARDUINO
#include "Arduino.h"

#ifdef RFDUINO
#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>
#else
#include <SPI.h>
// This lib had the .h and .cpp tweaked to disable debug and remove hard coded debug
#include <Adafruit_BLE_UART.h>
#endif

#endif

#endif

#include "NTProtocol.h"
#include "NTMessaging.h"


void     ble_setup();
void     ble_loop();

