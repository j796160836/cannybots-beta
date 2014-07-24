#ifndef NTutils_h
#define NTutils_h

//#include "NT_myconfig.h"
#include <stdint.h>

#if defined (ARDUINO) || defined (RFDUINO)
#include <Arduino.h>
template<class T> inline Print &operator <<(Print &obj, T arg) {  obj.print(arg);  return obj; }

#endif

// EEPROM (NV) 
#define NT_NV_CFG_BASE 10
#define NT_NV_CFG_DEVICE_ID 0


#define bytesFromInt(x)   (uint8_t)(x & 0xff), (uint8_t)((x &0xff00) >>8)
#define hiByteFromInt(x)  (uint8_t)((x &0xff00) >>8)
#define loByteFromInt(x)  (uint8_t)(x & 0xff)
#define mk16bit(lo,hi) ( (lo&0xFF) + ((hi&0xFF)<<8))

void NT_nv_init();
bool NT_nv_setupConfig();
bool NT_nv_isValidConfig();
bool NT_nv_setByte(uint16_t address, uint8_t b);
uint8_t NT_nv_getByte(uint16_t address);
bool NT_nv_setInt(uint16_t address, uint16_t b);
uint16_t NT_nv_getInt(uint16_t address);

void nv_cfg_set_deviceId(int16_t p);
int16_t nv_cfg_get_deviceId();


#if defined(NT_PLATFORM_AVR)
#include <Arduino.h>

#ifdef RFDUINO
#else
#include <EEPROM.h>
#endif


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define ATMEGA 1
#endif


#endif



/////////////////////////////////////////////////////
// Debug

#define DBG Serial.println


extern int voltage;

long readVcc();
int freeRam ();
void sysbanner();

#endif

