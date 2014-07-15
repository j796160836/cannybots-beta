//
//  CannybotsTypes.h
//
//  Created by Wayne Keenan on 09/07/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#ifndef _CannybotsTypes_h
#define _CannybotsTypes_h

#define CB_MAX_MSG_SIZE 20

typedef uint8_t cb_publish_type ;
typedef uint8_t cb_type ;



// TODO: create an class that has contructos
// RACER_SET_SPEED(1,2,3); // use eitehr a contructor on cb_id or a new Invoke class creted in the macro to make it look like a function call!


typedef struct {
    uint8_t  cid;
    char*    name;
} cb_id;


#ifdef ARDUINO
#define CB_ID(_cid, _idVarName, _name) \
const static cb_id _idVarName = {_cid};
// ignore the string to save SRAM and PRG space.
#else
 #define CB_ID(_cid, _idVarName,  _name) \
 const static cb_id _idVarName = { .cid = _cid, .name = _name };
#endif



typedef struct {
    cb_id    cid;
    cb_type  type;
    void*    data;
    uint16_t size;
    
    bool     isNV;
    bool     isMethod;
    
    bool     isPublished;
    cb_publish_type pubType;
    
    // should go elsewhere, attached to a dedicated list of published variables (along with a copy of pubtype)
    void*    lastData;
    unsigned long lastChangeTime;
    
} cb_descriptor;


#ifdef __cplusplus
extern "C" {
#endif
    // Callback prototypes
    typedef void (*cb_callback_int16_int16_int16) (int16_t p1, int16_t p2, int16_t p3);
    typedef void (*cb_callback_int16_int16) (int16_t p1, int16_t p2);
    typedef void (*cb_callback_int16) (int16_t p1);
    typedef void (cb_callback_gui) ();
    
#ifdef __cplusplus
}
#endif

#endif
