//
//  CannybotsTypes.h
//
//  Created by Wayne Keenan on 09/07/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

// This is a C only header, no C++ or Obj-C
#ifndef _CannybotsTypes_h
#define _CannybotsTypes_h

#include <CannybotsConfig.h>



typedef uint8_t cb_publish_type ;
typedef uint8_t cb_type ;



// TODO: create an class that has contructos
// RACER_SET_SPEED(1,2,3); // use eitehr a contructor on cb_id or a new Invoke class creted in the macro to make it look like a function call!


typedef struct cb_id_t{
    uint8_t  cid;
#ifndef ARDUINO
    const char*    name;
#endif
} cb_id;


typedef struct cb_nv_id_t {
    uint8_t  cid;
    uint16_t  offset;
#ifndef ARDUINO
    const char*    name;
#endif
} cb_nv_id;



// these 'compound literals' to save space (prmarily for  ARDUINO)
#ifdef ARDUINO
#define CB_ID(_cid, _idVarName, _name) \
static cb_id  _idVarName = {.cid= CB_MAX_SYS_DESCRIPTORS+_cid};
// ignore the string to save SRAM and PRG space.
#else
#define CB_ID(_cid, _idVarName,  _name) \
static cb_id _idVarName = { .cid = CB_MAX_SYS_DESCRIPTORS+_cid, .name = _name };
#endif



#ifdef ARDUINO
#define CB_NV_ID(_cid, _idVarName, _name, _structName, _structMember) \
static cb_nv_id _idVarName = {.cid = CB_MAX_SYS_DESCRIPTORS+_cid, .offset = offsetof(_structName, _structMember)};
// ignore the string to save SRAM and PRG space.
#else
#define CB_NV_ID(_cid, _idVarName,  _name, _structName, _structMember) \
static cb_nv_id _idVarName = { .cid = CB_MAX_SYS_DESCRIPTORS+_cid, .name = _name, .offset = offsetof(_structName, _structMember) };
#endif



typedef struct {
    union cid_u {
        cb_id*     cidMT;
        cb_nv_id*  cidNV;
    } cid_t;
    cb_type  type;
    void*    data;
    
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

#define _CANNYBOTS_SYSCALL_NOP                0
#define _CANNYBOTS_SYSCALL_GET_DEVICE_ID      1
#define _CANNYBOTS_SYSCALL_GET_VERSION        2
#define _CANNYBOTS_SYSCALL_GET_APP_CONFIG_ID  3
#define _CANNYBOTS_SYSCALL_GET_APP_CONFIG_VER 4



#endif
