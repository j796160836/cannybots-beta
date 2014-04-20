/*
  Copyright (C) 2014 Wayne Keenan  All rights reserved.
*/

#include "config.h"
#include "utils.h"
#include "NTProtocol.h"

#ifdef USE_BLE
#include <SPI.h>
// This lib had the .h and .cpp tweaked to disable debug and remove hard coded debug
#include <Adafruit_BLE_UART.h>
Adafruit_BLE_UART BLESerial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);


//////////////////////////////////////////////////////////////////////////////////////////////////
// BLE Call backs


void ble_setup() {
  BLESerial.setRXcallback(rxCallback);
  BLESerial.setACIcallback(aciCallback);
  BLESerial.begin();
}

void aciCallback(aci_evt_opcode_t event)
{
  switch (event)
  {
    case ACI_EVT_DEVICE_STARTED:
      brainAwake();
      break;
    case ACI_EVT_CONNECTED:
      clientConnected();
      break;
    case ACI_EVT_DISCONNECTED:
      clientDisconnected();
      break;
    default:
      break;
  }
}

void rxCallback(uint8_t *buffer, uint8_t len) {
  //MAYBE- TODO: toggle command processing on/off for temporary actions, such as raw data transfer?
  processMessage(buffer, len);
}


#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// NT Implementation specifics

// add a message to be sent to the client, currently sends immediately
// MAYBE-TODO: q it?
uint8_t msgQ[NT_MSG_SIZE];
uint8_t  NT_scheduleMsg(uint8_t* buffer) {
#ifdef USE_BLE
  BLESerial.write(buffer, NT_MSG_SIZE);
#endif
}






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
    case NT_CAT_SYSTEM:  
      // TODO: system commands
      break;
    case NT_CAT_NOP:  
      break;
    default:
      debug("Invalid category: %x", cat);
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
// AX12 


#ifdef USE_AX12
#include <ax12.h>

// Missing in 3rd party library:
#define ERR_NONE                    0
#define ERR_VOLTAGE                 1
#define ERR_ANGLE_LIMIT             2
#define ERR_OVERHEATING             4
#define ERR_RANGE                   8
#define ERR_CHECKSUM                16
#define ERR_OVERLOAD                32
#define ERR_INSTRUCTION             64


byte ax12_detect[AX12_MAX_SERVOS];
AX12 ax12_motors[AX12_MAX_SERVOS];
byte ax12_num = 0;
AX12 ax12_bcast_motor = AX12();



void ax12_setup() {
  
  AX12::init (1000000);   // AX12 at 1 Mb/s
  ax12_autodetect();
}


uint8_t ax12_autodetect() {
   ax12_num = AX12::autoDetect (ax12_detect, AX12_MAX_SERVOS);
  int m1id = ax12_num > 0 ? ax12_detect[0] : 1;
  int m2id = ax12_num > 1 ? ax12_detect[1] : 2;
  ax12_motors[0] = AX12((byte)m1id);
  ax12_motors[1] = AX12((byte)m2id, true);   // true for inverted commands, motors are facing each other

  ax12_motors[0].setEndlessTurnMode(true);
  ax12_motors[1].setEndlessTurnMode(true);
  ax12_motors[0].endlessTurn(0);
  ax12_motors[1].endlessTurn(0);
}

uint8_t ax12_getDiscoveredServoIdForLogicalID(uint8_t id) {
  return ax12_motors[id].id;
}


uint8_t  ax12_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  uint8_t servoId = ax12_getDiscoveredServoIdForLogicalID(id);
  AX12 motor = ax12_motors[servoId];
  switch (cmd) {
    case NT_CMD_AX12_PING:
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_RESET:
      motor.reset();
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ENDLESS_TURN_MODE:
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ENDLESS_TURN_SPEED:
      ax12_motors[0].endlessTurn(p1);
      ax12_motors[1].endlessTurn(p1);
      ax12_motors[0].endlessTurn(p1);
      ax12_motors[1].endlessTurn(p1);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_VELPOS:
      motor.setPos(p1);
      motor.setVel(p1);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ID:
       for (int i=1;  i< 254; i++) {
           motor = AX12(i);
           motor.changeID(id);
       }
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_DISCOVER:
      ax12_autodetect();
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_TESTS:
      status = ax12_runTests(id, p1);
      break;
    default:
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}


uint8_t  ax12_runTests(int id, int p1) {
  AX12 motor = ax12_motors[0];
  motor.setSRL(RETURN_ALL);
  AX12info result;
  result = motor.readInfo (RETURN_DELAY_TIME);
  byte val = result.value;
  int  err = result.error;
  debug("val=%x,err=%x", val, err);
  debug("ptr=%x,ax[0]=%x", AX12::ax_rx_Pointer, AX12::ax_rx_buffer[0]);
  //;
  //;
  
}
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Sonar 



#ifdef USE_SONAR
#include <NewPing.h>

// This library wwas modified to stop the usage of TIMER@ which conflist wijt the IRremote library
// see:https://code.google.com/p/arduino-new-ping/wiki/HELP_Error_Vector_7_When_Compiling
//



NewPing sonars[2] = {NewPing (TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE), NewPing(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE)};


uint8_t  sonar_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  switch (cmd) {
    case NT_CMD_SONAR_PING:
      status = sonar_generateReadings(id);
      break;
    default:
      DBG("sonar:NT_ERROR_CMD_INVALID!");
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}

uint8_t sonar_ping(uint8_t id) {
  unsigned int uS = sonars[ id == 1 ? 0 : 1].ping();
  return uS / US_ROUNDTRIP_CM;   // distance in whole CM's
}

// when id=0 => all sensors,
uint8_t sonar_generateReadings(uint8_t id) {
  uint8_t status;
  if (0 == id) {
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, NT_CMD_NO_CONT), 1, bytesFromInt(sonar_ping(1)),
      NT_CREATE_CMD(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, NT_CMD_NO_CONT), 2, bytesFromInt(sonar_ping(2)),
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    status = NT_scheduleMsg(msg);

  } else {
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, NT_CMD_NO_CONT), id, bytesFromInt(sonar_ping(id)),
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    status = NT_scheduleMsg(msg);

  }
  return status;
}

#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Servos

#ifdef USE_SERVOS
#include <Servo.h>
Servo servos[SERVO_MAX_SERVOS];
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Motors

#ifdef USE_MOTORS
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// IR Receiver

#ifdef USE_IRRECV
#include <IRremoteORG.h>

IRrecv irrecv(IRRECV_RECV_PIN); 

void irrecv_init() {
  irrecv.enableIRIn();
}


void irrecv_dump(decode_results *results);
decode_results irrecv_results;
uint8_t irrecv_generateReadings(uint8_t id)  {     // 0 =all,  1 = 1st, 2 = 2nd etc
  uint8_t status = NT_STATUS_OK;
  
  if (irrecv.decode(&irrecv_results)) {
    //int count = irrecv_results.rawlen;
    //irrecv_results.decode_type;    // manufacturrer code define'd constant (SONY, PANASONIC, RC5, RC6, et.c. see header/example)
    //3irrecv_results.value;          // key press (
    //irrecv_results.panasonicAddress
    
    //debug("v=%x t=%x", irrecv_results.value, irrecv_results.decode_type, irrecv_results.panasonicAddress);
    irrecv.resume(); // Receive the next value
  
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD(NT_CAT_IRRECV, NT_CMD_IRRECV_RESULT, NT_CMD_NO_CONT), 1, bytesFromInt(irrecv_results.value),
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    status = NT_scheduleMsg(msg);
  }
  return status;
}


#endif

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



//////////////////////////////////////////////////////////////////////////////////////////////////
//
// System Events

void brainAwake() {
}

void clientConnected() {
  INFO_PRINTLN((""));
  INFO_PRINTLN(("        **** CANNY BRAIN V2 ****"));
  INFO_PRINTLN(("  2K RAM SYSTEM  "));
  INFO_PRINTLN(freeRam());
  INFO_PRINTLN((" BRAIN BYTES FREE"));
  INFO_PRINTLN((""));
  INFO_PRINTLN(("READY."));
}

void clientDisconnected() {
}

void emitSensorData() {
  // TODO: read from EEPROM config what to send.
#ifdef USE_SONAR
  sonar_generateReadings(0); // 0 =all,  1 = 1st, 2 = 2nd
#endif
#ifdef USE_IRRECV
  irrecv_generateReadings(0); // 0 =all,  1 = 1st, 2 = 2nd

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

  debug_setup();
}


void loop() {
#ifdef USE_BLE
  BLESerial.pollACI();
#endif
  int analogreportnums = ANALOG_REPORT_NUM;
  samplingInterval = (uint16_t)MINIMUM_SAMPLE_DELAY  + (uint16_t)ANALOG_SAMPLE_DELAY * (1 + analogreportnums);
  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    emitSensorData();
  }
}

