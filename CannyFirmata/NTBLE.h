#ifndef NTBLE_H
#define NTBLE_H

#include <Arduino.h>

#include "NTProtocol.h"



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

extern SimpleFIFO<BLEMessage*, 4> inboundMsgFIFO;
extern SimpleFIFO<BLEMessage*, 4> outboundMsgFIFO;  
  

void ble_setup();
void ble_loop();
uint8_t  NT_scheduleMsg(uint8_t* buffer);
void     NT_processInboundMessageQueue();
void     NT_processOutboundMessageQueue();



//////////////////////////////////////////////////////////////////////////////////////////////////
// BLE Implementation specifics

#ifdef RFDUINO
#include <RFduinoBLE.h>
#else
#include <SPI.h>
// This lib had the .h and .cpp tweaked to disable debug and remove hard coded debug
//#include <Adafruit_BLE_UART.h>

#endif



#endif
