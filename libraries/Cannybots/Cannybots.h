/*********************************************************************
 *********************************************************************/

#ifndef _CANNYBOTS_H
#define _CANNYBOTS_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __APPLE__
#include "TargetConditionals.h"
#include <iostream>
#endif

#define DEBUG 1
#define INFO_LED 13
#ifdef DEBUG
static char dbg_buffer[128];
#ifdef ARDUINO
#define LOG(FMT, ...) snprintf(dbg_buffer, 128, FMT, __VA_ARGS__); Serial1.println(dbg_buffer);
#else 
#define LOG(FMT, ...) snprintf(dbg_buffer, 128, FMT, __VA_ARGS__); std::cout << dbg_buffer;
#endif //ARDUINO
#else
#define LOG(...)
#endif


#ifdef ARDUINO
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#endif // ARDUINO


class cb_id;


extern "C"
{
    // Callback prototypes
    typedef void (*cb_callback_int16_int16) (int16_t p1, int16_t p2);
    typedef void (cb_callback_gui) ();
}

typedef uint8_t cb_publish_type ;

typedef struct {
    cb_id*   _id;
    void*   data;
} cb_descriptor;



#define CB_MAX_DESCRIPTORS 16


class cb_id {
public:
    cb_id(uint16_t _cid): cid(_cid)
#ifndef ARDUINO
    ,name(NULL)
#endif
    {
        //assignId();
    };
    
#ifndef ARDUINO
    cb_id(const char* _name): cid(0), name(NULL)  {
        if (name)
            free(name);
        name = strdup(_name);
        strcpy(name, _name);
        //assignId();
    };
#endif
    ~cb_id() {
#ifndef ARDUINO
        if (name)
            free(name);
#endif
    }

    void assignId() {
        cid=cb_id::nextId++;
        LOG("CID=%d", cid);
    }
    
private:
    uint16_t cid;
#ifndef ARDUINO
    char*    name;
#endif
    static uint16_t nextId;
};

#ifdef ARDUINO
#define CB_ATTRIBUTE_ID(_idVarName, _cid, _name) \
const static cb_id _idVarName(_cid);
// ignore the string to save SRAM and PRG space.
#else
#define CB_ATTRIBUTE_ID(_idVarName, _cid, _name) \
const static cb_id _idVarName(_cid, _name);
#endif




class Cannybots
{
private:
    cb_descriptor descriptors[CB_MAX_DESCRIPTORS];
    
public:
    
    const static cb_publish_type PUBLISH_UPDATE_ONCHANGE;
    
    Cannybots ();
    
    void setConfigStorage(const char* magic, const uint16_t start);
    
    // Scalar
    void registerVariable(const cb_id& _id, int16_t* var, const bool isNonVolatile=false, const int16_t defaultValue=0);
    void registerVariable(const cb_id& _id, bool*    var, const bool isNonVolatile=false, const bool    defaultValue=false);
    
    // Arrays
    void registerArray(const cb_id _id, int16_t list[], const uint16_t length);
    
    // Published values
    void registerPublisher(const cb_id _id, bool *var, const cb_publish_type pubType);

    
    // Function handlers
    void registerHandler(const cb_id _id, cb_callback_int16_int16);
    
    // Scripting
    
    void registerScritableVariable(const cb_id _id, const char* name);
#ifdef ARDUINO
    void registerScritableVariable(const cb_id _id, const __FlashStringHelper*);
#endif
    
    // GUI
    void gui_addButton(const int16_t x, const int16_t y, const char* buttonText,  cb_callback_gui callback);
    void gui_addLevelMeter(const int16_t x, const int16_t y, const char* label,  int16_t* variable, const int16_t min, const int16_t max);

    
    // utils
    bool validate();
    void begin();
    void update();

    uint16_t getLastError();
    /*
     getters & setters for:
        DeviceID
        DeviceName
         TXPower
         DebugLevel
     
     
     ReadOnly:
         BatteryLevel
         FreeMemory
         DebugLogs
     
     utils for:
        resetConfig
     
     */

private:
    bool         debug;
    
    
};

#endif
