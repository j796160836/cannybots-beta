
#include <Arduino.h>
#include "NTBLE.h"

SimpleFIFO<BLEMessage*, 4> inboundMsgFIFO;
SimpleFIFO<BLEMessage*, 4> outboundMsgFIFO;  

uint8_t  NT_scheduleMsg(uint8_t* buffer) {
  BLEMessage* msg = new BLEMessage(buffer, NT_MSG_SIZE);
  outboundMsgFIFO.enqueue(msg);
}

void NT_processInboundMessageQueue() {
  
  for (int i=0; i<inboundMsgFIFO.count(); i++) {
    BLEMessage* msg = inboundMsgFIFO.dequeue();
    processMessage(msg->payload, msg->size);
    delete msg;
  }
}


void brainAwake() {
  //DBG("brainAwake\n");
}

void clientConnected() {
  //INFO_PRINTLN("CON\n");
}

void clientDisconnected() {
  //INFO_PRINTLN("DISCON\n");
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// RFDuino Specifics
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
// Shared functions

void ble_setup() {
#ifdef RFDUINO
  RFduinoBLE.customUUID = "7e400001-b5a3-f393-e0a9-e50e24dcca9e";
  RFduinoBLE.begin();
#else
  BLESerial.setRXcallback(rxCallback);
  BLESerial.setACIcallback(aciCallback);
  BLESerial.begin();
#endif
}

void ble_loop() {
  NT_processOutboundMessageQueue();
#ifdef RFDUINO
  // no need todo anything.
#else
  BLESerial.pollACI();
#endif
  NT_processInboundMessageQueue();
}



void NT_processOutboundMessageQueue() {  
  for (int i=0; i<outboundMsgFIFO.count(); i++) {
    BLEMessage* msg = outboundMsgFIFO.dequeue();
#ifdef RFDUINO
    RFduinoBLE.send((char*)msg->payload, NT_MSG_SIZE);
#else
    BLESerial.write(msg->payload, NT_MSG_SIZE);
#endif
    delete msg;
  }
}

