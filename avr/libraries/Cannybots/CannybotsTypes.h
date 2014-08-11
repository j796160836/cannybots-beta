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

typedef struct cb_id_t{
    uint8_t  cid;
#ifndef ARDUINO
    const char*    name;
#endif
} cb_id;



// TODO: need to warn if offset is > 255
typedef struct cb_nv_id_t {
    uint8_t  cid;
#ifndef ARDUINO
    const char*    name;
#endif
} cb_nv_id;



typedef struct {
    union cid_u {
        cb_id*     cidMT;
        cb_nv_id*  cidNV;
    } cid_t;
    cb_type  type;
    void*    data;
} cb_descriptor;


// TODO:  published vars, see below
//bool     isPublished;
//cb_publish_type pubType;
//void*    lastData;
//unsigned long lastChangeTime;




#ifdef ARDUINO
#define CB_ID(_cid, _idVarName, _name) \
static cb_id  _idVarName = {.cid= CB_MAX_SYS_DESCRIPTORS+_cid};
// ignore the string to save SRAM and PRG space.
#else
#define CB_ID(_cid, _idVarName,  _name) \
static cb_id _idVarName = { .cid = CB_MAX_SYS_DESCRIPTORS+_cid, .name = _name };
#endif



#ifdef ARDUINO
#define CB_CFG_ID(_structMember) \
static cb_nv_id _structMember = {.cid= offsetof(cb_app_config, _structMember)};
// ignore the string to save SRAM and PRG space.
#else
#define CB_CFG_ID(_structMember) \
static cb_nv_id _structMember = { .cid = offsetof(cb_app_config, _structMember), .name = #_structMember};
#endif



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
CB_ID(0-CB_MAX_SYS_DESCRIPTORS, _CB_SYS_CALL, "syscall");       // multi-purpose system call, first uint8_t param is function select, following bytes will depend on func.
CB_ID(1-CB_MAX_SYS_DESCRIPTORS, _CB_SYS_LOG, "log");            // log a message (may be truncated to 20 bytes)

#define _CB_SYSCALL_NOP                  0
#define _CB_SYSCALL_GET_BOT_ID           1
#define _CB_SYSCALL_GET_BOT_TYPE         2
#define _CB_SYSCALL_GET_CFG_VERSION      3
#define _CB_SYSCALL_GET_CFG_LIST         4
#define _CB_SYSCALL_GET_CFG_PARAM        5
#define _CB_SYSCALL_SET_CFG_PARAM        6
#define _CB_SYSCALL_CFG_PARAM_META       7


#endif
