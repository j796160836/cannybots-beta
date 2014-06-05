#include <stdarg.h>

#include "NTUtils.h"


int voltage=0;



void   debug_setup() {
#if defined(INFO_OUT_TARGET_UART)
   Serial.begin(INFO_OUT_TARGET_UART_BAUD);
#endif
   
}



#if defined (NT_PLATFORM_AVR)
void debug(char * format, ...)
{
  char buffer[32];
  va_list args;
  va_start (args, format);
  vsnprintf (buffer, 31, format, args);
  va_end (args);
  DBG(buffer);
}

void debug(const __FlashStringHelper* format, ...)
{
  char buffer[32];
  va_list args;
  va_start (args, format);
  vsnprintf (buffer, 31, reinterpret_cast<const char*>(format), args);
  va_end (args);
  DBG(buffer);
}

// limit the number of writes, just in case we get stuck in a loop during dev!
int NT_nv_writesSinceBoot=0;
bool NT_nv_setByte(uint16_t address, uint8_t b) {
  NT_nv_writesSinceBoot++;
  if (NT_nv_writesSinceBoot < 1000) {
/*    INFO_PRINT("eeprom[");
    INFO_PRINT(address);
    INFO_PRINT("]=");
    INFO_PRINTLN(b);
*/
    EEPROM.write(address, b);
  } else {
    INFO_PRINTLN("exceeded max EEPROM writes for this session");
  }
}

uint8_t NT_nv_getByte(uint16_t address) {
  EEPROM.read(address);
}


bool NT_nv_setInt(uint16_t address, uint16_t b) {
    uint8_t b1 = hiByteFromInt(b);
    uint8_t b2 = loByteFromInt(b);
/*    
    INFO_PRINTLN("-----");
    INFO_PRINT("NV write 16 bit:");
    INFO_PRINT(address);
    INFO_PRINT("=");
    INFO_PRINTLN(b);
    INFO_PRINT("HI=");
    INFO_PRINTLN(b1);
    INFO_PRINT("LO=");
    INFO_PRINTLN(b2);
*/
    NT_nv_setByte(address,b1);
    NT_nv_setByte(address+1,b2);
}

uint16_t NT_nv_getInt(uint16_t address) {
    uint8_t b1 = NT_nv_getByte(address);
    uint8_t b2 = NT_nv_getByte(address+1);
    int16_t b = mk16bit(b2,b1);
    
/*
    INFO_PRINTLN("-----");
    INFO_PRINT("NV READ 16bit:");
    INFO_PRINT(address);
    INFO_PRINT("=");
    
    INFO_PRINTLN(b);
    INFO_PRINT("HI=");
    INFO_PRINTLN(b1);
    INFO_PRINT("LO=");
    INFO_PRINTLN(b2);
 */
    return b;
}


bool NT_nv_setupConfig() {
  NT_nv_setByte(0, 'C');
  NT_nv_setByte(1, 'N');
  NT_nv_setByte(2, 'Y');
  NT_nv_setByte(3, 'B');
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
     NT_nv_configDefaults_LineFollowing();
   } else {
//      debug("NV is CNYB\n");
   }
}


void nv_cfg_set_deviceId(int16_t p) {
   NT_nv_setInt(NT_NV_CFG_BASE + NT_NV_CFG_DEVICE_ID, p);
}
int16_t nv_cfg_get_deviceId() {
   NT_nv_getInt(NT_NV_CFG_BASE + NT_NV_CFG_DEVICE_ID);
}



#else

void debug(char * format, ...)
{
    
}
#endif

void hexDump(uint8_t *data, uint16_t len) {
  uint8_t cnt = 0;
  for (int i = 0; i < len; i++) {
    printf("%.2x ", data[i]);
    cnt++;
    if (cnt == 8) {
      printf(" ");
    }
    if (cnt == 16) {
      printf("\n");
      cnt = 0;
    }
  }
  if (cnt != 0) {
    printf("\n");
  }
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

