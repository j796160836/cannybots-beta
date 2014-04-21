#ifndef utils_h
#define utils_h
#include <Arduino.h>

#include "config.h"

template<class T> inline Print &operator <<(Print &obj, T arg) {  obj.print(arg);  return obj; }


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define ATMEGA 1
#endif

/////////////////////////////////////////////////////
// Debug
void   debug_setup();
void debug(char * format, ...);
void debug(const __FlashStringHelper* format, ...);


#ifdef DEBUG
#ifdef USE_BLE
#define DBG(x) BLESerial.println(x);
#else
#define DBG(x) Serial.println(x);
#endif
#define DBG_PRINT(x) DBG_OUT << x
#define DBG_PRINT_DETAILED(x) DBG_OUT <<  " : " << __PRETTY_FUNCTION__ << ' ' << __FILE__ << ":" << __LINE__  << " - "  << x
#define DBG_PRINT_SIMPLE(x) DBG_OUT << x
#define DBG_PRINTLN(x) DBG_PRINT_SIMPLE(x) << "\n"
#else
#define DBG(x)
#define DBG_PRINT(x)
#define DBG_PRINTLN(x)
#endif

#ifdef INFO
#define INFO(x) INFO_PRINT(x)
#define INFO_PRINT(x) INFO_OUT << x
#define INFO_PRINT_DETAILED(x) INFO_OUT <<  " : " << __PRETTY_FUNCTION__ << ' ' << __FILE__ << ":" << __LINE__  << " - "  << x
#define INFO_PRINT_SIMPLE(x) INFO_OUT << x
#define INFO_PRINTLN(x) INFO_PRINT_SIMPLE(x) << "\n"
#else
#define INFO(x)
#define INFO_PRINT(x)
#define INFO_PRINTLN(x)
#endif

#define mk16bit(x,y) ( x + (y<<8))

extern int voltage;

long readVcc();
int freeRam ();

#endif;
