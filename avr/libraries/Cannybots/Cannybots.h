//
//  Cannybots.h
//
//  Created by Wayne Keenan
//  Copyright (c) 2014 CannyBots. All rights reserved.
//


#ifndef _CANNYBOTS_H
#define _CANNYBOTS_H

// Message format

// 20 bytes
// msg[0]     = 'C'
// msg[1]     = 'B'
// msg[2]     = 1 byte CRC
// msg[3]     = seq[7..4] .. type[3...0]
// msg[4]     = cmd
// msg[5]     = param count (MAYBE TODO: > max len implies mulit packet)
// msg[6-7]   = int16
// msg[8-9]   = int16
// msg[10-11] = int16
// msg[12-13] = int16
// msg[14-15] = int16
// msg[16-17] = int16
// msg[18-19] = int16


/* type:
 
 byte/char  = 1 byte
 uint/int   = 2 bytes
 ulong/long = 4 bytes
 float      = 4 bytes
 
 0  void
 1  scalar: byte
 2  scalar: int
 3  scalar: uint
 4  scalar: long
 5  scalar: ulong
 6  scalar: float
 7  array:  byte[]               - arbitrary data, e.g. custom binary protocol such as EasyTransfer
 8 string (0 terminated)
 9 int16[7]                 // short
 10 uint16[7]                // word
 11 long32[3]
 12 ulong32[3]
 13 float32[3]
 14 int16,int16,int16
 15 float,float,float
 
 */

// Adding a new function signature for RPC:

// 1. add the function prototype to 'Callback prototypes' below         (e.g. cb_callback_string)
// 2. ensure there is a CB constatnce in Cannybots.h   'Callback parameter prototype'  (e.g. CB_STRING)
// 3. add a registerHandler to Cannybots.{h|cpp}  e.g. void Cannybots::registerHandler(const cb_id& _id, cb_callback_string callback) {../}

// 4. in cannybots.[h] add a callMethod(type)   e.g.  void callMethod(cb_id cid, const char* p1)

// 5. in cannybots.h maybe create a helper:     void createMessage(Message* msg, cb_id cid, const char* p1)

// 6. in Cannybots::processMessage(Message* msg ) add an inbound handler.

// 4. (iOS) add brding prototype in CannybotsContrller.h , see  'for ObjC / C++ blocks'   (e.g. typedef void (^cb_bridged_callback_string)(const char*);
// 5. (iOS) add to CannybotsContrller.mm    e.g   - (void) registerHandler:(cb_id)cid withBlockFor_CB_STRING:(cb_callback_string)block { }
// 6. (iOS) add the onReceive hfunciton handler to CannyBotsController.mm - (void) didReceiveData:(NSData *)data {



#include <CannybotsConfig.h>


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>     /* offsetof */

#ifdef __APPLE__
#include "TargetConditionals.h"
#include <iostream>
// for free ram
#import <mach/mach.h>
#import <mach/mach_host.h>
#endif // __APPLE__


#ifdef ARDUINO
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#endif // ARDUINO


#include <CannybotsTypes.h>
#include <CannybotsUtils.h>
#include <CannybotsInt.h>

#define CB_REGISTER_CONFIG(_p)    Cannybots::getInstance().registerConfigParameter(&_p, &settings->_p);

#define _CB_TEMPLATE_registerConfigParameter(_ctype, _cb_type) \
void Cannybots::registerConfigParameter(cb_nv_id* _id, _ctype *v) { \
CB_DBG("Reg:%d type=%d", _id->cid, _cb_type);\
cb_descriptor* desc =  CB_ALLOC_DESC(); \
desc->cid_t.cidNV = _id; \
desc->type = _cb_type; \
desc->data = v; \
configVars.add(desc); }

class Cannybots
{
public:
    // Type MetaInfo for: Message parameter type, Config type and Callback method signatures (up to 255)
    const static cb_type CB_VOID;
    const static cb_type CB_BYTE;
    const static cb_type CB_INT8;
    const static cb_type CB_UINT8;
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
    const static cb_type CB_INT16_1;
    const static cb_type CB_INT16_2;
    const static cb_type CB_INT16_3;
    const static cb_type CB_FLOAT_1;
    const static cb_type CB_FLOAT_2;
    const static cb_type CB_FLOAT_3;
    
    const static cb_publish_type PUBLISH_UPDATE_ONCHANGE;
    
    
    static Cannybots& getInstance()
    {
        // not instatiated on first use, instead do 'controlled' setup in Cannybots::begin()
        // the definition is in the .cpp because AVR compilation targert complains about missing ATEXIT used for dtors of static vars in function.
        // see: http://forum.arduino.cc/index.php/topic,73177.0.html
        return instance;
    }

    
    // Scalar
    void registerVariable(cb_id* _id, int16_t* var, const bool isNonVolatile=false, const int16_t defaultValue=0);
    void registerVariable(cb_id* _id, bool*    var, const bool isNonVolatile=false, const bool    defaultValue=false);
    
    // Arrays
    void registerArray(cb_id* _id, int16_t list[], const uint16_t length);
    
    // Published values
    void registerPublisher(cb_id* _id, bool *var, const cb_publish_type pubType);

    
    // Function handlers
    void registerHandler(cb_id* _id, cb_callback_int16_int16_int16);
    void registerHandler(cb_id* _id, cb_callback_int16_int16);
    void registerHandler(cb_id* _id, cb_callback_int16);
    void registerHandler(cb_id* _id, cb_callback_string);
    
    // 'generic' callback poitners (e.g. iOSblocks)
    void registerHandler(cb_id* _id, uint8_t type, void* callback);

    
    void deregisterHandler( cb_id* _id);

    
    // Scripting
    
    void registerScritableVariable(cb_id* _id, const char* name);
#ifdef ARDUINO
    void registerScritableVariable(cb_id* _id, const __FlashStringHelper*);
    
    unsigned long getLastInboundCommandTime() {
        return lastInboundMessageTime;
    }
#endif
    
    // GUI
    void gui_addButton(const int16_t x, const int16_t y, const char* buttonText,  cb_callback_gui callback);
    void gui_addLevelMeter(const int16_t x, const int16_t y, const char* label,  int16_t* variable, const int16_t min, const int16_t max);

    
    // Comms
    
    void updateVariable(cb_id* _id, bool value);
    
    
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
    
    void addInboundMessage(Message* msg)  { inboundMsgFIFO.enqueue(msg);  }
    
    // TODO: make sending immediately a configuration option
    void addOutboundMessage(Message* msg) { outboundMsgFIFO.enqueue(msg); }
    
    
    void sendMessage(Message* msg);

    
    // Message Creation Helpers
    // TODO generalise, and make use of para count
    void createMessage(Message* msg, cb_id* cid, int16_t p1, int16_t p2, int16_t p3) ;
    void createMessage(Message* msg, cb_id* cid, int16_t p1, int16_t p2);
    void createMessage(Message* msg, cb_id* cid, int16_t p1);
    void createMessage(Message* msg, cb_id* cid, const char* p1)    ;
    
    void createMessage(Message* msg, cb_id* cid, int16_t p1, int16_t p2, int16_t p3, uint32_t ui32);

    
    // Remote invocation entry point
    
    void callMethod(cb_id* cid, int16_t p1);
    void callMethod(cb_id* cid, int16_t p1,int16_t p2,int16_t p3);
    void callMethod(cb_id* cid, const char* p1);     // String (null terminated) type


    cb_descriptor* getDescriptorForCommand(uint8_t commandId);


    
    
    // Non-Volatile Configuration Setting (e.g. EEPROM on Arduono, FLASH on RFduino)

    
    void setConfigStorage(const char* magic, const uint16_t start, const uint16_t size, uint8_t majorVersion, uint8_t minorVersion);

    void registerConfigParameter(cb_nv_id* _id, bool* v);
    void registerConfigParameter(cb_nv_id* _id, int8_t* v);
    void registerConfigParameter(cb_nv_id* _id, uint8_t* v);
    void registerConfigParameter(cb_nv_id* _id, int16_t* v);
    void registerConfigParameter(cb_nv_id* _id, uint16_t* v);
    void registerConfigParameter(cb_nv_id* _id, int32_t* v);
    void registerConfigParameter(cb_nv_id* _id, uint32_t* v);
  
/*
    void setConfigParameter(cb_descriptor* desc, bool value);
    void setConfigParameter(cb_descriptor* desc, int8_t value);
    void setConfigParameter(cb_descriptor* desc, uint8_t value);
    void setConfigParameter(cb_descriptor* desc, int16_t value);
    void setConfigParameter(cb_descriptor* desc, uint16_t value);
    void setConfigParameter(cb_descriptor* desc, int32_t value);
    void setConfigParameter(cb_descriptor* desc, uint32_t value);
*/
    
    // these function just work of the cid info (e.g. struct memebr offset) direct to/from EEPROM
    // they dont know the location of the user sketch variable (e.g. the data pointed to by a descriptor, i.e. the 'settings' struct data) )
    void getConfigParameterValue(cb_nv_id* _id, int8_t* v);
    void setConfigParameterValue(cb_nv_id* _id, int8_t* v);
    void getConfigParameterValue(cb_nv_id* _id, uint8_t* v);
    void setConfigParameterValue(cb_nv_id* _id, uint8_t* v);
    void getConfigParameterValue(cb_nv_id* _id, int16_t* v);
    void setConfigParameterValue(cb_nv_id* _id, int16_t* v);
    void getConfigParameterValue(cb_nv_id* _id, uint16_t* v);
    void setConfigParameterValue(cb_nv_id* _id, uint16_t* v);
    void getConfigParameterValue(cb_nv_id* _id, int32_t* v);
    void setConfigParameterValue(cb_nv_id* _id, int32_t* v);
    void getConfigParameterValue(cb_nv_id* _id, uint32_t* v);
    void setConfigParameterValue(cb_nv_id* _id, uint32_t* v);
    void getConfigParameterValue(cb_nv_id* _id, bool* v);
    void setConfigParameterValue(cb_nv_id* _id, bool* v);


    void populateVariablesFromConfig();

    void            getConfigParameterListFromRemote();
    uint16_t        getConfigParameterListSize();
    cb_descriptor*  getConfigParameterListItem(int16_t index);

    void setConfigParameter(uint8_t cid, uint32_t value);
    void sendConfigParameterList();

    cb_descriptor* getDescriptorForConfigParameter(uint8_t cid);

    void dumpConfig();

private:
    static Cannybots instance; // Guaranteed to be destroyed. (yeah, when the power goes lol)

    // Singleton
    Cannybots () {
        lastInboundMessageTime = 0;
    };
    Cannybots(Cannybots const&);        // Don't Implement
    void operator=(Cannybots const&);   // Don't implement
    
    
    void registerSysCalls();

    // Message processing
    void processInboundMessageQueue();
    void processOutboundMessageQueue();
    void processMessage(Message* msg);

    CBFIFO<Message*, CB_MAX_IN_Q_DEPTH>  inboundMsgFIFO;
    CBFIFO<Message*, CB_MAX_OUT_Q_DEPTH> outboundMsgFIFO;
    
    LinkedList<cb_descriptor*>  methods    = LinkedList<cb_descriptor*>();
    LinkedList<cb_descriptor*>  configVars = LinkedList<cb_descriptor*>();

    int16_t getIndexForDescriptor(LinkedList<cb_descriptor*> list, cb_descriptor* desc);

    uint16_t nvBaseAddress;
    
    // utils
    bool         debug;
    

    
#ifdef ARDUINO
    void readSerial(HardwareSerial &ser);
#endif
    unsigned long lastInboundMessageTime;

};

#endif
