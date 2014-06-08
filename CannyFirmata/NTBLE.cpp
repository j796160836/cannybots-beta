
#include <Arduino.h>
#include "NT_myconfig.h"
#include "NTBLE.h"
#include "NTUtils.h"

// share state
SimpleFIFO<BLEMessage*, 4> inboundMsgFIFO;
SimpleFIFO<BLEMessage*, 4> outboundMsgFIFO;

boolean volatile ble_connected = false;
boolean volatile gzll_connected = false;


// Shared functions, with no BLE device specifc implementation details

uint8_t  NT_scheduleMsg(uint8_t* buffer) {
  BLEMessage* msg = new BLEMessage(buffer, NT_MSG_SIZE);
  outboundMsgFIFO.enqueue(msg);
}

void NT_processInboundMessageQueue() {

  for (int i = 0; i < inboundMsgFIFO.count(); i++) {
    BLEMessage* msg = inboundMsgFIFO.dequeue();
    processMessage(msg->payload, msg->size);
    delete msg;
  }
}


void brainAwake() {
  DBG("brainAwake\n");
}

void clientConnected() {
  DBG("CON\n");
  ble_connected = true;
}

void clientDisconnected() {
  DBG("DISCON\n");
  ble_connected = false;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// RFDuino specifc Callbacks
#ifdef RFDUINO

void RFduinoBLE_onAdvertisement(bool start) {
  brainAwake();
}

void RFduinoBLE_onConnect() {
  clientConnected();
}

void RFduinoBLE_onDisconnect() {
  clientDisconnected();
}


void RFduinoBLE_onReceive(char *data, int len) {
  BLEMessage* msg = new BLEMessage((uint8_t*)data, len);
  inboundMsgFIFO.enqueue(msg);
}

// GZLL

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  DBG("RFduinoGZLL_onReceive");
  gzll_connected = true;
  
  char state = data[0];
  pinMode(3, OUTPUT);
  digitalWrite(3, state);
  // no data to piggyback on the acknowledgement sent back to the Device
  RFduinoGZLL.sendToDevice(device, "OK");
}



// swap between BLE and GZLL
#define BLE_MAX_TICKS        10240
#define BLE_MAX_TICKS_HALF   5120

void  ble_rfduino_manageRadios() {

  static int ticks = 0;
  if (ble_connected) 
    return;
  if (gzll_connected)
    return;
    
  if ( 1 == ticks ) {
    DBG("BLE\n");
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    
    RFduinoGZLL.end();
    RFduinoBLE.customUUID = "7e400001-b5a3-f393-e0a9-e50e24dcca9e";
    RFduinoBLE.begin();

  } else if ( BLE_MAX_TICKS_HALF == ticks){
    DBG("GZLL\n");
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    while(!RFduinoBLE.radioActive);
    while(RFduinoBLE.radioActive);
    RFduinoBLE.end();
    while(RFduinoBLE.radioActive);
    RFduinoGZLL.begin(HOST);
  } else {
     //DBG(ticks, DEC); 
  }
  //delay(5);
  ticks = (ticks+1 ) % BLE_MAX_TICKS;
}

#else
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Adafruit CAllbacks
Adafruit_BLE_UART BLESerial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

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
  BLEMessage* msg = new BLEMessage(buffer, len);
  inboundMsgFIFO.enqueue(msg);
}

#endif



//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Shared functions, with BLE device dependant implementation details



void ble_setup() {
#ifdef RFDUINO
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(5, LOW);  digitalWrite(6, LOW);  delay(500);
  digitalWrite(5, HIGH);  digitalWrite(6, LOW);  delay(500);
  digitalWrite(5, HIGH);  digitalWrite(6, HIGH);  delay(500);
  digitalWrite(5, LOW);  digitalWrite(6, HIGH);  delay(500);
  digitalWrite(5, LOW);  digitalWrite(6, LOW);  delay(500);

  //DBG("ble_setup: The device id is:");
  //uint64_t id = getDeviceId();
  //DBG(getDeviceIdLow(), HEX);
  //DBG(getDeviceIdHigh(), HEX);
#else
  BLESerial.setRXcallback(rxCallback);
  BLESerial.setACIcallback(aciCallback);
  BLESerial.begin();
#endif
}

void ble_loop() {
  NT_processOutboundMessageQueue();
#ifdef RFDUINO
  ble_rfduino_manageRadios();
#else
  BLESerial.pollACI();
#endif
  NT_processInboundMessageQueue();
}



void NT_processOutboundMessageQueue() {
  for (int i = 0; i < outboundMsgFIFO.count(); i++) {
    BLEMessage* msg = outboundMsgFIFO.dequeue();
#ifdef RFDUINO
    RFduinoBLE.send((char*)msg->payload, NT_MSG_SIZE);
#else
    BLESerial.write(msg->payload, NT_MSG_SIZE);
#endif
    delete msg;
  }
}

