/*
  Copyright (C) 2014 Cannybots  All rights reserved.
*/


// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#define nop() __asm volatile ("nop")
#if 1
nop();
#endif
//void setup(); // stop Arduiono C pre-processor screwyness, see http://forum.arduino.cc/index.php/topic,38052.0.html




#include <Arduino.h>

#include "NT_myconfig.h"
#include "NTUtils.h"
#include "NTProtocol.h"
#include "NT_App_LineFollowing.h"



#ifdef RFDUINO
#else
    #include <Arduino.h>
    #include <EEPROM.h>
#endif

#ifdef RFDUINO
#include <RFduinoBLE.h>
#else
#include <SPI.h>
// This lib had the .h and .cpp tweaked to disable debug and remove hard coded debug
//#include <Adafruit_BLE_UART.h>

#endif

#ifdef USE_BLE
#include "NTBLE.h"
#endif



//////////////////////////////////////////////////////////////////////////////////////////////////
// Command Processing


int  processCategoryCont(int cat, int cmd, int id, int p1, int p2, int p3) {
}



uint8_t  processCategory(int cat, int cmd, int id, int p1) {
  uint8_t status = NT_STATUS_OK;
  switch (cat) {
#ifdef USE_AX12
    case NT_CAT_AX12:
      status = ax12_processCommand(cmd, id, p1);
      break;
#endif     
#ifdef USE_SONAR      
    case NT_CAT_SONAR:
      status = sonar_processCommand(cmd, id, p1);
      break;
#endif      
#ifdef USE_TONE
    case NT_CAT_TONE:
      status = tone_processCommand(cmd, id, p1);
      break;
#endif
    //case NT_CAT_APP:  
    case NT_CAT_APP_LINEFOLLOW:
      status = linefollow_processCommand(cmd, id, p1);
      
      break;
    case NT_CAT_NOP:  
      break;
    default:
      //debug(F("Invalid category: %x"), cat);
      status=NT_ERROR_CAT_INVALID;
  }
  return status;
}




void processMessage(uint8_t *buffer, uint8_t len)
{
  uint8_t status = NT_validateMsg(buffer, len);

  if ( NT_STATUS_OK == status ) {

    // loop thru the commands
    for (int p = NT_MSG_HEADER_BYTES; p < NT_MSG_SIZE; p += NT_MSG_COMMAND_BYTES) {
      int  cat  = NT_CAT_FROM_MSG_BYTE(buffer[p]);
      int  cmd  = NT_CMD_FROM_MSG_BYTE(buffer[p]);
      
      if (NT_CAT_NOP == cat) 
        continue;
        
      if (! NT_CONT_FROM_MSG_BYTE(buffer[p])) {
        status = processCategory(cat, cmd, buffer[p + 1], mk16bit(buffer[p + 2], buffer[p + 3]));
        if (NT_STATUS_OK != status) {
          NT_sendError(status);
        }      
      } else {
        status = processCategoryCont(cat, cmd, buffer[p + 1],
                            mk16bit(buffer[p + 2], buffer[p + 3]),
                            mk16bit(buffer[p + 4], buffer[p + 5]),
                            mk16bit(buffer[p + 6], buffer[p + 7]));
        if (NT_STATUS_OK != status) {
          NT_sendError(status);
        }
        p += NT_MSG_COMMAND_BYTES;
      }
    }
  } else {
    NT_sendError(status);
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////
//
// System Commands

uint8_t  system_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  switch (cmd) {
    case NT_CMD_SYS_INFO_BANNER:
    /*
      DBG(("Brain is alive!"));
      DBG(("Voltage: %d mv") << voltage);
      DBG(("Detected: ") << ax12_num);
      DBG(("ping 1: ") << ax12_motors[0].ping() );
      DBG(("ping 2: ") << ax12_motors[1].ping() );
    */
      status = NT_STATUS_OK;
      break;
    default:
      DBG("system:NT_ERROR_CMD_INVALID!");
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}


void emitSensorData() {
  // TODO: read from EEPROM config what to send.
#ifdef USE_SONAR
  sonar_generateReadings(0); // 0 =all,  1 = 1st, 2 = 2nd
#endif
#ifdef USE_IRRECV
  irrecv_generateReadings(0); // 0 =all,  1 = 1st, 2 = 2nd

#endif

#ifdef USE_MIC
  mic_generateReadings(0); // 0 =all,  1 = 1st, 2 = 2nd
#endif
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Low Level System


/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
int samplingInterval = 200;          // how often to run the main loop (in ms)

void sys_setup() {
  voltage = readVcc();

}


void setup() {
  
  sys_setup();
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

  debug_setup();
  NT_nv_init();
 
  lf_init();
 /*
  INFO_PRINTLN(F(""));
  INFO_PRINTLN(F("    **** CANNYBOTS BRAIN V2 ****"));
  INFO_PRINT(F("  2K RAM SYSTEM  "));
  INFO_PRINT(freeRam());
  INFO_PRINTLN(F(" BRAIN BYTES FREE"));
  INFO_PRINTLN(F(""));
  INFO_PRINTLN(F("READY."));
  */
}


void loop() {
#ifdef USE_BLE
  ble_loop();
#endif

  lf_loop();
  
#ifdef USE_PIXY
   updatePixy();
#endif

  int analogreportnums = ANALOG_REPORT_NUM;
  samplingInterval = (uint16_t)MINIMUM_SAMPLE_DELAY  + (uint16_t)ANALOG_SAMPLE_DELAY * (1 + analogreportnums);
  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    emitSensorData();
  }
}

