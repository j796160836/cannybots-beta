/*
  Copyright (C) 2014 Wayne Keenan  All rights reserved.
*/

#include "NTUtils.h"
#include "NTProtocol.h"

#include <Arduino.h>
#include <EEPROM.h>


// Inbound & outbound FIFO's
#include "SimpleFIFO.h"
class BLEMessage {
  public:
    BLEMessage(uint8_t* buffer, uint16_t len) {
      memcpy(payload, buffer, NT_MSG_SIZE);
      size=len;
    };
  uint8_t payload[NT_MSG_SIZE];
  uint8_t size;
};

SimpleFIFO<BLEMessage*, 4> inboundMsgFIFO;
SimpleFIFO<BLEMessage*, 4> outboundMsgFIFO;  
  

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
  //
  BLEMessage* msg = new BLEMessage(buffer, len);
  
  inboundMsgFIFO.enqueue(msg);
}


#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// NT Implementation specifics



uint8_t  NT_scheduleMsg(uint8_t* buffer) {
  BLEMessage* msg = new BLEMessage(buffer, NT_MSG_SIZE);
  outboundMsgFIFO.enqueue(msg);

}

void NT_processOutboundMessageQueue() {
  
  for (int i=0; i<outboundMsgFIFO.count(); i++) {
    BLEMessage* msg = outboundMsgFIFO.dequeue();
#ifdef USE_BLE
    BLESerial.write(msg->payload, NT_MSG_SIZE);
#endif
    delete msg;
  }
}


void NT_processInboundMessageQueue() {
  
  for (int i=0; i<inboundMsgFIFO.count(); i++) {
    BLEMessage* msg = inboundMsgFIFO.dequeue();
    processMessage(msg->payload, msg->size);
    delete msg;
  }
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
byte ax12_num;



void ax12_setup() {
  
  AX12::init (1000000);   // AX12 at 1 Mb/s
  ax12_autodetect();
  ax12_defaults();
}

void ax12_defaults() {
  
  for (int i =0; i< ax12_num; i++) {
    if (i<ax12_num) {
      int mid =  ax12_detect[i];
      ax12_motors[i].id=mid;
      ax12_motors[i].setEndlessTurnMode(true);
      ax12_motors[i].endlessTurn(0);
    }
  }
  if (ax12_num>=2)
    ax12_motors[1].inverse=true;   
}

uint8_t ax12_autodetect() {
   ax12_num = AX12::autoDetect (ax12_detect, AX12_MAX_SERVOS);
   debug(F("num=%d"), ax12_num);
}


uint8_t  ax12_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  

  AX12* motor = &ax12_motors[id-1];
  
  switch (cmd) {
    case NT_CMD_AX12_PING:
      status = motor->ping()==0?NT_STATUS_OK:NT_ERROR_CMD_GENERIC_ERROR;
      break;
    case NT_CMD_AX12_RESET:
      motor->reset();
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ENDLESS_TURN_MODE:
      motor->setEndlessTurnMode(p1>0?true:false);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ENDLESS_TURN_SPEED:
      motor->endlessTurn(p1);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_POS:
      motor->setPos(p1);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_VEL:
      motor->setVel(p1);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_VELPOS:
      // TODO: move to multi arg command..
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ID:
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_DISCOVER:
      ax12_autodetect();
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_TESTS:
      status = ax12_runTests(id, p1);
      break;
    //case NT_CMD_SCAN_RESET:
    //   for (int i=1;  i< 254; i++) {
    //       AX12 m = AX12(id)
    //       m->changeID(id);
    //   }
    
    default:
      status = NT_ERROR_CMD_INVALID;
  }
  
  return status;
}

uint8_t  ax12_runTests(int id, int p1) {
  //AX12 motor = ax12_motors[0];
  ax12_motors[0].setSRL(RETURN_ALL);
  AX12info result;
  result = ax12_motors[0].readInfo (RETURN_DELAY_TIME);
  byte val = result.value;
  int  err = result.error;
  debug("val=%x,err=%x", val, err);
  debug("ptr=%x,ax[0]=%x", AX12::ax_rx_Pointer, AX12::ax_rx_buffer[0]);


  delay(15);
  // ping all +1 for an error ping value
  for (byte i = 0 ; i< 5; i++) {
    AX12 m = AX12(i+1);
    debug("p(%d,%x)=%x", i,  m.id,  m.ping());
  }


  ax12_defaults();
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
      NT_CREATE_CMD1(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, 1, sonar_ping(1)),
      NT_CREATE_CMD1(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, 2, sonar_ping(2)),
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
// including Servo.h causes clash of timers with Tone lib!! 
// even if USE_SERVO isnt defiend!!  Arduiono is scann thi ssource without accounting for that
//#include <Servo.h>
//Servo servos[SERVO_MAX_SERVOS];
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

// this library was moifed, name change to resolve conflit with standard library (which had classh with other lib..)
// modief alto to use TIMER1 and not timer 2 in IRRemoteIntORG.h
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
    //irrecv_results.decode_type;    // manufacturrer code define'd constant (e,g,SONY, PANASONIC, RC5, RC6, et.c. see header/example)
    //3irrecv_results.value;          // key press (
    //irrecv_results.panasonicAddress
    
    debug("v=%x t=%x\n", irrecv_results.value, irrecv_results.decode_type, irrecv_results.panasonicAddress);
    irrecv.resume(); // Receive the next value
  
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      //TODO: use the cont and send all three values
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
// Pixy Commands





#ifdef USE_PIXY
#include <Wire.h>
#include <PixyI2C.h>
PixyI2C pixy;

void setup_pixy() {
}

void updatePixy() {
  debug(F("Detected %d"), pixy.getBlocks());
}

#endif




//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Tone Commands



#ifdef USE_TONE
#include <avr/pgmspace.h>

#include "pitches.h"
PROGMEM prog_uint16_t melody[] = { NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
PROGMEM prog_uint16_t noteDurations[] = { 4, 8, 8, 4,4,4,4,4 };

uint8_t tone_playDitty(int p1) {
 for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/pgm_read_word_near(noteDurations+thisNote);
    tone(TONE_PIN, pgm_read_word_near(melody+thisNote),noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(TONE_PIN);
  }  
  return NT_STATUS_OK;
}


uint8_t tone_playTone(uint8_t id, int p1) {
    int noteDuration = 1000/id; // 4 or 8 - see ply ditty
    tone(TONE_PIN, p1, noteDuration);  // playDitt / pitces.h
  return NT_STATUS_OK;
}


uint8_t  tone_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  switch (cmd) {
    case NT_CMD_TONE_PLAY_DITTY:
      status = tone_playDitty(id);
      break;
    case NT_CMD_TONE_PLAY_TONE:
      status = tone_playTone(id, p1);
      break;
    default:
      DBG("XX:NT_ERROR_CMD_INVALID!");
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}

#endif 


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// XXX Commands


/*
#ifdef USE_XXX
uint8_t  XXX_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  switch (cmd) {
    case NT_CMD_XXXCAT_XXXMD:
      status = XXCAT_XXX(id);
      break;
    default:
      DBG("XX:NT_ERROR_CMD_INVALID!");
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}

uint8_t XXX_cmd(uint8_t id) {
  unsigned int uS = sonars[ id == 1 ? 0 : 1].ping();
  return uS / US_ROUNDTRIP_CM;   // distance in whole CM's
}


// when id=0 => all sensors,
uint8_t XXX_generateReadings(uint8_t id) {
  uint8_t status;
  if (0 == id) {
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD1(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, 1, sonar_ping(1)),
      NT_CREATE_CMD1(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, 2, sonar_ping(2)),
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
*/


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Mic
// when id=0 => all sensors,

#ifdef USE_MIC
uint8_t mic_generateReadings(uint8_t id) {
  uint8_t status;
  if (0 == id) {
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD1(NT_CAT_MIC, NT_CMD_MIC_RESULT, 1, analogRead(MIC_PIN)),
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
  //DBG("brainAwake\n");
}

void clientConnected() {
  //DBG("clientConnected\n");
}

void clientDisconnected() {
  //DBG("clientDisconnected\n");
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
  NT_processOutboundMessageQueue();

  #ifdef USE_BLE
  BLESerial.pollACI();
#endif
  NT_processInboundMessageQueue();

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

