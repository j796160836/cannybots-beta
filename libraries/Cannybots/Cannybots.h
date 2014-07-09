/*********************************************************************
 *********************************************************************/

#ifndef _CANNYBOTS_H
#define _CANNYBOTS_H

#define DEBUG 1
#define INFO_LED 13

// Transport buffers
#define SERIAL_BUF_SIZE 32
#define CB_MAX_OUT_Q_DEPTH 4
#define CB_MAX_IN_Q_DEPTH 4

// Connection settings
#define CB_INBOUND_SERIAL_PORT Serial1
#define CB_INBOUND_SERIAL_BAUD 9600


// Message payload offsets
#define CB_MSG_OFFSET_CMD  4
#define CB_MSG_OFFSET_DATA 6

// Exchanged variables info
#define CB_MAX_DESCRIPTORS 16


// helpers
#define hiByteFromInt(x)  (uint8_t)((x &0xff00) >>8)
#define loByteFromInt(x)  (uint8_t)(x & 0xff)
#define mk16bit(lo,hi) ( (lo&0xFF) + ((hi&0xFF)<<8))


#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "CannybotsTypes.h"


#ifdef DEBUG
#ifdef ARDUINO
static char dbg_buffer[128];
#define CB_DBG(FMT, ...) snprintf(dbg_buffer, 128, FMT, __VA_ARGS__); Serial.println(dbg_buffer);
#else
static char dbg_buffer[256];
#define CB_DBG(FMT, ...) printf("implement iOS log\n"); //printf(FMT, __VA_ARGS__);
#endif //ARDUINO
#else // DEBUG
#define LOG(...)
#endif




template<typename T, int rawSize>
class CBFIFO {
public:
	const char size;				//speculative feature, in case it's needed
    
	CBFIFO();
    
	T dequeue();				//get next element
	bool enqueue( T element );	//add an element
	T peek() const;				//get the next element without releasing it from the FIFO
	void flush();				//[1.1] reset to default state
    
	//how many elements are currently in the FIFO?
	char count() { return numberOfElements; }
    
private:
#ifndef SimpleFIFO_NONVOLATILE
	volatile char numberOfElements;
	volatile char nextIn;
	volatile char nextOut;
	volatile T raw[rawSize];
#else
	char numberOfElements;
	char nextIn;
	char nextOut;
	T raw[rawSize];
#endif
};

template<typename T, int rawSize>
CBFIFO<T,rawSize>::CBFIFO() : size(rawSize) {
	flush();
}
template<typename T, int rawSize>
bool CBFIFO<T,rawSize>::enqueue( T element ) {
	if ( count() >= rawSize ) { return false; }
	numberOfElements++;
	nextIn %= size;
	raw[nextIn] = element;
	nextIn++; //advance to next index
	return true;
}
template<typename T, int rawSize>
T CBFIFO<T,rawSize>::dequeue() {
	numberOfElements--;
	nextOut %= size;
	return raw[ nextOut++];
}
template<typename T, int rawSize>
T CBFIFO<T,rawSize>::peek() const {
	return raw[ nextOut % size];
}
template<typename T, int rawSize>
void CBFIFO<T,rawSize>::flush() {
	nextIn = nextOut = numberOfElements = 0;
}





#ifdef __APPLE__
#include "TargetConditionals.h"
#include <iostream>
// for free ram
#import <mach/mach.h>
#import <mach/mach_host.h>

#endif


#ifdef ARDUINO
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#endif // ARDUINO


class Message {
public:
    Message(): size(0) {
    };
    Message(uint8_t* buffer, uint16_t len) {
        memcpy(payload, buffer, len<CB_MAX_MSG_SIZE?len:CB_MAX_MSG_SIZE);
        size=len;
    };
    uint8_t payload[CB_MAX_MSG_SIZE];
    uint8_t size;
};



class Cannybots
{
public:
    //TODO: private
    cb_descriptor descriptors[CB_MAX_DESCRIPTORS];
    
public:
    
    const static cb_publish_type PUBLISH_UPDATE_ONCHANGE;
    
    const static cb_type CB_VOID;
    const static cb_type CB_BYTE;
    const static cb_type CB_INT16;
    const static cb_type CB_UINT16;
    const static cb_type CB_INT32;
    const static cb_type CB_UINT32;
    const static cb_type CB_INT64;
    const static cb_type CB_UINT64;
    const static cb_type CB_FLOAT;
    const static cb_type CB_BYTE_ARRAY;
    const static cb_type CB_STRING;
    const static cb_type CB_INT16_ARRAY;
    const static cb_type CB_UINT16_ARRAY;
    const static cb_type CB_INT32_ARRAY;
    const static cb_type CB_UINT32_ARRAY;
    const static cb_type CB_FLOAT_ARRAY;
    const static cb_type CB_INT16_3;
    const static cb_type CB_FLOAT_3;

    
    static Cannybots instance; // Guaranteed to be destroyed.
    static Cannybots& getInstance()
    {
        // not instatiated on first use, do 'controlled' setup in Cannybots::begin()
        // the definition is in th e.cpp because AVR compilation targer complains about missing ATEXIT
        // see: http://forum.arduino.cc/index.php/topic,73177.0.html
        return instance;
    }

    void setConfigStorage(const char* magic, const uint16_t start);
    
    // Scalar
    void registerVariable(const cb_id& _id, int16_t* var, const bool isNonVolatile=false, const int16_t defaultValue=0);
    void registerVariable(const cb_id& _id, bool*    var, const bool isNonVolatile=false, const bool    defaultValue=false);
    
    // Arrays
    void registerArray(const cb_id _id, int16_t list[], const uint16_t length);
    
    // Published values
    void registerPublisher(const cb_id _id, bool *var, const cb_publish_type pubType);

    
    // Function handlers
    void registerHandler(const cb_id _id, cb_callback_int16_int16_int16);
    
    
    // 'generic' callback poitners (e.g. iOSblocks)
    void registerHandler(const cb_id _id, uint8_t type, void* callback);

    
    
    // Scripting
    
    void registerScritableVariable(const cb_id _id, const char* name);
#ifdef ARDUINO
    void registerScritableVariable(const cb_id _id, const __FlashStringHelper*);
#endif
    
    // GUI
    void gui_addButton(const int16_t x, const int16_t y, const char* buttonText,  cb_callback_gui callback);
    void gui_addLevelMeter(const int16_t x, const int16_t y, const char* label,  int16_t* variable, const int16_t min, const int16_t max);

    
    // Comms
    
    void updateVariable(const cb_id& _id, bool value) {
        
    }
   
    
    
    // utils
    bool validate();
    void begin();
    void update();

    uint16_t getLastError();
    
    //      getters & setters for:
    const char* getDeviceId();
    const char* getDeviceName();
    int16_t getTXPowerLevel();
    int16_t getDebugLevel();
    
    
    // READ ONLY
    int16_t getBatteryLevel();
    int16_t getFreeMemory();
    void getDebugLogs();

    // Utils
    
    void resetNVConfiguration();

    // public queue methods
    
    void addInboundMessage(Message* msg) {
        inboundMsgFIFO.enqueue(msg);
    }
    void addOutboundMessage(Message* msg) {
        outboundMsgFIFO.enqueue(msg);
    }
    
    
    
    // Message Creation Helpers
    
    void createMessage(Message* msg, cb_id cid, int16_t p1, int16_t p2, int16_t p3) {
        uint8_t tmpMsg[CB_MAX_MSG_SIZE] = {
            'C', 'B', 0,
            Cannybots::CB_INT16_3, cid.cid,
            3,
            hiByteFromInt(p1),loByteFromInt(p1),
            hiByteFromInt(p2),loByteFromInt(p2),
            hiByteFromInt(p3),loByteFromInt(p3),
            0,0,  0,0,  0,0, 0,0
        };
        memcpy((char*)msg->payload, (char*)tmpMsg, CB_MAX_MSG_SIZE);
        msg->size = CB_MAX_MSG_SIZE;
    }
    
    
    // REmote invocation
    
    void callMethod(cb_id cid, int16_t p1) {
        Message* msg = new Message();
        createMessage(msg, cid, p1, 0, 0);
        addOutboundMessage(msg);
    }
    

    
    
private:
    
    // Singleton
    Cannybots () {};
    Cannybots(Cannybots const&);        // Don't Implement
    void operator=(Cannybots const&);   // Don't implement
    
    // Message processing
    void processInboundMessageQueue();
    void processOutboundMessageQueue();
    void processMessage(Message* msg);

    CBFIFO<Message*, CB_MAX_IN_Q_DEPTH>  inboundMsgFIFO;
    CBFIFO<Message*, CB_MAX_OUT_Q_DEPTH> outboundMsgFIFO;
    

    // utils
    bool         debug;
    
#ifdef ARDUINO
    // serial
    uint8_t serialBuffer[SERIAL_BUF_SIZE+1];
    int serialBufPtr;
    bool foundStart;
    char c, lastChar;
    void readSerial(HardwareSerial &ser);
#endif

};

#endif
