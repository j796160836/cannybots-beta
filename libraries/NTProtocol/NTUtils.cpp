#ifdef ARDUINO
#include <Arduino.h>
#ifdef RFDUINO
#else
#include <EEPROM.h>
#endif
#endif

#include <stdarg.h>
#include "NTUtils.h"
//#include "NT_myconfig.h"


int voltage=0;

// limit the number of writes, just in case we get stuck in a loop during dev!
int NT_nv_writesSinceBoot=0;
bool NT_nv_setByte(uint16_t address, uint8_t b) {
  NT_nv_writesSinceBoot++;
  if (NT_nv_writesSinceBoot < 1000) {

#ifdef ARDUINO

#ifdef RFDUINO

// see:  https://github.com/RFduino/RFduino/tree/master/libraries/RFduinoNonBLE/examples/Flash/FlashInteger
#else
    EEPROM.write(address, b);
#endif
#endif
  } else {
    //WARN("exceeded max EEPROM writes for this session");
  }
    return 0;
}

uint8_t NT_nv_getByte(uint16_t address) {
    uint8_t b = 0;
#ifdef ARDUINO

#ifdef RFDUINO
// see:  https://github.com/RFduino/RFduino/tree/master/libraries/RFduinoNonBLE/examples/Flash/FlashInteger
#else
  b = EEPROM.read(address);
#endif
#endif
    return b;
}


bool NT_nv_setInt(uint16_t address, uint16_t b) {
    uint8_t b1 = hiByteFromInt(b);
    uint8_t b2 = loByteFromInt(b);

    NT_nv_setByte(address,b1);
    NT_nv_setByte(address+1,b2);
    return 0;
}

uint16_t NT_nv_getInt(uint16_t address) {
    uint8_t b1 = NT_nv_getByte(address);
    uint8_t b2 = NT_nv_getByte(address+1);
    int16_t b = mk16bit(b2,b1);
    return b;
}


bool NT_nv_setupConfig() {
  NT_nv_setByte(0, 'C');
  NT_nv_setByte(1, 'N');
  NT_nv_setByte(2, 'Y');
  NT_nv_setByte(3, 'B');
    return 0;
}

bool NT_nv_isValidConfig() {
  uint8_t b1 = NT_nv_getByte(0);
  uint8_t b2 = NT_nv_getByte(1);
  uint8_t b3 = NT_nv_getByte(2);
  uint8_t b4 = NT_nv_getByte(3);
  
  return (  (b1 == 'C') && (b2 == 'N') && (b3 == 'Y') && (b4 == 'B') );
}

void NT_nv_configDefaults_LineFollowing();

void NT_nv_init() {
 //   debug("NV INIT\n");

   if (!NT_nv_isValidConfig()) {
     //delay(5000);
  //   debug("NVsetup\n");
     NT_nv_setupConfig();
#ifdef ARDUINO
       
     NT_nv_configDefaults_LineFollowing();
#endif
       
   } else {
//      debug("NV is CNYB\n");
   }
}


void nv_cfg_set_deviceId(int16_t p) {
   NT_nv_setInt(NT_NV_CFG_BASE + NT_NV_CFG_DEVICE_ID, p);
}
int16_t nv_cfg_get_deviceId() {
   return NT_nv_getInt(NT_NV_CFG_BASE + NT_NV_CFG_DEVICE_ID);
}



long readVcc() {
  long result = -1;
  

#if defined (NT_PLATFROM_AVR)
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

    result = (high << 8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
#endif  
  return result; // Vcc in millivolts
}


int freeRam () {
#if defined (NT_PLATFORM_AVR)
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
#else 
    return -1;
#endif
}

void sysbanner(){
   /*
  INFO_PRINTLN(F(""));
  INFO_PRINTLN(F("    **** CANNYBOTS BRAIN V2 ****"));
  INFO_PRINT(F("  2K RAM SYSTEM  "));
  INFO_PRINT(freeRam());
  INFO_PRINTLN(F(" BRAIN BYTES FREE"));
  INFO_PRINTLN(F(""));
  INFO_PRINTLN(F("READY."));
  */
}

