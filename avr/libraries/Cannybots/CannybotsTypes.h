//
//  CannybotsTypes.h
//
//  Created by Wayne Keenan on 09/07/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//


// Adding a new type:

// 1. add the function prototype to 'Callback prototypes' below         (e.g. cb_callback_string)
// 2. ensure there is a CB constatnce in Cannybots.h   'Callback parameter prototype'  (e.g. CB_STRING)
// 3. add a registerHandler to Cannybots.{h|cpp}  e.g. void Cannybots::registerHandler(const cb_id& _id, cb_callback_string callback) {../}

// 4. in cannybots.[h] add a callMethod(type)   e.g.  void callMethod(cb_id cid, const char* p1)

// 5. in cannybots.h maybe create a helper:     void createMessage(Message* msg, cb_id cid, const char* p1)

// 4. (iOS) add brding prototype in CannybotsContrller.h , see  'for ObjC / C++ blocks'   (e.g. typedef void (^cb_bridged_callback_string)(const char*);
// 5. (iOS) add to CannybotsContrller.mm    e.g   - (void) registerHandler:(cb_id)cid withBlockFor_CB_STRING:(cb_callback_string)block { }
// 6. (iOS) add the onReceive hfunciton handler to CannyBotsController.mm - (void) didReceiveData:(NSData *)data {
  




#ifndef _CannybotsTypes_h
#define _CannybotsTypes_h

#define CB_MAX_MSG_SIZE 20


// Message payload offsets
#define CB_MSG_OFFSET_CMD  4
#define CB_MSG_OFFSET_DATA 6

// Exchanged variables info
#define CB_MAX_SYS_DESCRIPTORS 2
#define CB_MAX_DESCRIPTORS 12+CB_MAX_SYS_DESCRIPTORS



typedef uint8_t cb_publish_type ;
typedef uint8_t cb_type ;



// TODO: create an class that has contructos
// RACER_SET_SPEED(1,2,3); // use eitehr a contructor on cb_id or a new Invoke class creted in the macro to make it look like a function call!


typedef struct {
    uint8_t  cid;
#ifndef ARDUINO
    char*    name;
#endif
} cb_id;


#ifdef ARDUINO
#define CB_ID(_cid, _idVarName, _name) \
const static cb_id _idVarName = {CB_MAX_SYS_DESCRIPTORS+_cid};
// ignore the string to save SRAM and PRG space.
#else
 #define CB_ID(_cid, _idVarName,  _name) \
 const static cb_id _idVarName = { .cid = CB_MAX_SYS_DESCRIPTORS+_cid, .name = _name };
#endif




typedef struct {
    cb_id    cid;
    cb_type  type;
    void*    data;
    uint16_t size;
    
    //bool     isNV;
    bool     isMethod;
    
    //bool     isPublished;
    //cb_publish_type pubType;
    
    // should go elsewhere, attached to a dedicated list of published variables (along with a copy of pubtype)
    //void*    lastData;
    //unsigned long lastChangeTime;
    
} cb_descriptor;


#ifdef __cplusplus
extern "C" {
#endif
    // Callback prototypes
    typedef void (*cb_callback_int16_int16_int16) (int16_t p1, int16_t p2, int16_t p3);
    typedef void (*cb_callback_int16_int16) (int16_t p1, int16_t p2);
    typedef void (*cb_callback_int16) (int16_t p1);
    typedef void (*cb_callback_string) (const char* p1);
    typedef void (cb_callback_gui) ();
    
#ifdef __cplusplus
}
#endif


// system functions
// total cant be > CB_MAX_SYS_DESCRIPTORS
CB_ID(0, _CB_SYS_CALL, "syscall");       // multi-purpose system call, first uint8_t param is function select, following bytes will depend on func.
CB_ID(1, _CB_SYS_LOG, "log");            // log a message (may be truncated to 20 bytes)

#define _CANNYBOTS_SYSCALL_NOP              0
#define _CANNYBOTS_SYSCALL_GET_DEVICE_ID    1
#define _CANNYBOTS_SYSCALL_SET_DEVICE_ID    2
#define _CANNYBOTS_SYSCALL_GET_TIMESTAMP    3



#endif
