/* Copyright (C) 2014 Cannybots  All rights reserved.
*/


#include <Arduino.h>
#include "NT_myconfig.h"
#include "NTUtils.h"
#include "NTProtocol.h"
#include "NT_App_LineFollowing.h"
#include "NT_MOD_SensorCommon.h"

// Arduiono C pre-processing makes it necessary to manually commont and uncomment RFDuiono or Adafriot  (I suppose we could install the adafruit BLE lib in the RFDuiono toolchain, odd!)
#ifdef RFDUINO
#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>

#else
#include <SPI.h>
#include <Arduino.h>
#include <EEPROM.h>
// This lib had the .h and .cpp tweaked to disable debug and remove hard coded debug
//#include <Adafruit_BLE_UART.h>
#endif

#ifdef USE_BLE
#include "NTBLE.h"
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {  
  Serial.begin(9600 );
  DBG("started\n");
  
  //voltage = readVcc();
  
#ifdef USE_AX12  
  ax12_setup();
#endif

// NOTE:    init IR *BEFORE* BLE  - order is important  TODO: investigate possible timer clash and elimitate ay kind of 'working by luck'
#ifdef USE_IRRECV
  irrecv_init();
#endif

#ifdef USE_BLE
  ble_setup();
#endif
  
#ifdef USE_PIXY
  setup_pixy();
#endif


  NT_nv_init();
 
  lf_init();

  sysbanner();
}

//////////////////////////////

void loop() {
#ifdef USE_BLE
  ble_loop();
#endif

#ifdef USE_PIXY
   updatePixy();
#endif
  sensor_update();
  lf_loop();
}

