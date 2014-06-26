
#include "NTMessaging.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

// share state
NTQueue inboundMsgFIFO;
NTQueue outboundMsgFIFO;

// Shared functions, with no BLE device specifc implementation details


void NT_sendCommand(int8_t cat, uint8_t cmd, uint8_t _id, uint16_t p1) {
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD_NOP,//NT_CREATE_CMD(cat, cmd, NT_CMD_NO_CONT), _id, bytesFromInt(p1),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };

#if 0
//#ifdef ARDUINO
    Serial.print(cat, HEX);
    Serial.print(" ");
    Serial.print(cmd, HEX);
    Serial.print(" ");
    Serial.print(_id, HEX);
    Serial.print(" ");
    Serial.println(p1, HEX);
#endif
    NT_CREATE_CMD_WITH_ARG1((&msg[NT_MSG_HEADER_BYTES]), cat, cmd, _id, p1);
    
    /*
     msg[NT_MSG_HEADER_BYTES]=NT_CREATE_CMD(cat, cmd, 0);
     msg[NT_MSG_HEADER_BYTES+1]=_id;
     msg[NT_MSG_HEADER_BYTES+2]=hiByteFromInt(p1);
     msg[NT_MSG_HEADER_BYTES+3]=hiByteFromInt(p1);
     */
    NT_MSG_CALC_CRC(msg);
    
    NT_scheduleMsg(msg);
}




uint8_t  NT_scheduleMsg(uint8_t* buffer) {
  BLEMessage* msg = new BLEMessage(buffer, NT_MSG_SIZE);
  outboundMsgFIFO.enqueue(msg);
    return NT_STATUS_OK;
}

void NT_processInboundMessageQueue() {

  for (int i = 0; i < inboundMsgFIFO.count(); i++) {
    BLEMessage* msg = inboundMsgFIFO.dequeue();
    processMessage(msg->payload, msg->size);
    delete msg;
  }
}



void NT_processOutboundMessageQueue() {
#ifdef ARDUINO
    //Serial.print("Out FIFO COUNT");
    //Serial.println(outboundMsgFIFO.count(), DEC);
#endif
    for (int i = 0; i < outboundMsgFIFO.count(); i++) {
        BLEMessage* msg = outboundMsgFIFO.dequeue();
#ifdef ARDUINO
#ifdef USE_BLE

#ifdef RFDUINO
        RFduinoBLE.send((char*)msg->payload, NT_MSG_SIZE);
#else // adafruit
        BLESerial.write(msg->payload, NT_MSG_SIZE);
#endif
#else // UART
        
#ifdef ARDUINO_AVR_LEONARDO  // or a-star
        //Serial.println("SENDING!");
        Serial1.write(">>");
        Serial1.write(msg->payload, NT_MSG_SIZE);
#else
        Serial.write(">>");
        Serial.write(msg->payload, NT_MSG_SIZE);
#endif // ARDUINO_AVR_LEONARDO
        
#endif // USE_BLE
#else
        // Non-arduiono send...
        
#endif // ARDUINO
        
        delete msg;
    }
}


void NT_message_enqueueInboundMessage(uint8_t* data, uint16_t len) {
    BLEMessage* msg = new BLEMessage(data, len);
    inboundMsgFIFO.enqueue(msg);
}




#define SERIAL_BUF_SIZE 32
uint8_t serialBuffer[SERIAL_BUF_SIZE+1];
int serialBufPtr = 0;
bool foundStart = false;
char c=0, lastChar=0;

#ifdef ARDUINO
void NT_UART_parseIntoCallback(HardwareSerial &ser, messageHandler handler) {
//}

//void NT_UART_parseIntoQueue(HardwareSerial &ser, NTQueue &queue) {
    
    while (ser.available()>0) {
        lastChar = c;
        c =  ser.read();
        //Serial.print(" ");
        //Serial.print(c, HEX);
        
        if (foundStart && (serialBufPtr<SERIAL_BUF_SIZE)) {
            serialBuffer[serialBufPtr++] = c;
        } else if ( ('>' == c) && ('>' == lastChar) ) {
            foundStart=true;
            serialBufPtr=0;
        }
        
        if (serialBufPtr>=NT_MSG_SIZE) {
            //Serial.println("<EOC>");
            foundStart = false;
            //BLEMessage* msg = new BLEMessage(serialBuffer, NT_MSG_SIZE);
            //queue.enqueue(msg);
        handler(serialBuffer, NT_MSG_SIZE);

            serialBufPtr=0;
        }
    }
}
#endif
