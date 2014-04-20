#include <stdarg.h>

#include "utils.h"


int voltage=0;



void   debug_setup() {
   //Serial.begin(9600); 
}

void debug(char * format, ...)
{
  char buffer[32];
  va_list args;
  va_start (args, format);
  vsnprintf (buffer, 31, format, args);
  va_end (args);
  //DBG(buffer);
  BLESerial.print(buffer);
}


long readVcc() {
  // Read 1.1V reference against AVcc. Inverting that, means vcc becomes known!
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  // call this only in setup()
  // ref    http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

  uint8_t admux_0 = ADMUX;   // save it.   presumably =0?
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);   // aref=vcc read=1.1 internal
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  ADMUX = admux_0;   // restore

  long result = (high << 8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

