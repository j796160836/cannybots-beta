#ifndef NTMessaging_h
#define NTMessaging_h


#include <NTProtocol.h>
#include <SimpleFIFO.h>
#include <string.h>
// Inbound & outbound FIFO's
class BLEMessage {
  public:
    BLEMessage(uint8_t* buffer, uint16_t len) {
      memcpy(payload, buffer, NT_MSG_SIZE);
      size=len;
    };
  uint8_t payload[NT_MSG_SIZE];
  uint8_t size;
};

typedef SimpleFIFO<BLEMessage*, 4> NTQueue;
typedef SimpleFIFO<BLEMessage*, 4> NTQueue;

extern SimpleFIFO<BLEMessage*, 4> inboundMsgFIFO;
extern SimpleFIFO<BLEMessage*, 4> outboundMsgFIFO;  


uint8_t  NT_scheduleMsg(uint8_t* buffer);
void     NT_processInboundMessageQueue();
void     NT_processOutboundMessageQueue();

#ifdef ARDUINO
#include <Arduino.h>
void NT_UART_parseIntoQueue(HardwareSerial &ser, NTQueue &queue);
#endif

#endif

