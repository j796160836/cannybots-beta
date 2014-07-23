// ArduinoISP version 04m3
// Copyright (c) 2008-2011 Randall Bohn
// If you require a license, see
//     http://www.opensource.org/licenses/bsd-license.php
//
// This sketch turns the Arduino into a AVRISP
// using the following arduino pins:
//
// pin name:    not-mega:         mega(1280 and 2560)
// slave reset: 10:               53
// MOSI:        11:               51
// MISO:        12:               50
// SCK:         13:               52
//
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat   - shows the programmer is running
// 8: Error       - Lights up if something goes wrong (use red if that makes sense)
// 7: Programming - In communication with the slave
//
// 23 July 2011 Randall Bohn
// -Address Arduino issue 509 :: Portability of ArduinoISP
// http://code.google.com/p/arduino/issues/detail?id=509
//
// October 2010 by Randall Bohn
// - Write to EEPROM > 256 bytes
// - Better use of LEDs:
// -- Flash LED_PMODE on each flash commit
// -- Flash LED_PMODE while writing EEPROM (both give visual feedback of writing progress)
// - Light LED_ERR whenever we hit a STK_NOSYNC. Turn it off when back in sync.
// - Use pins_arduino.h (should also work on Arduino Mega)
//
// October 2009 by David A. Mellis
// - Added support for the read signature command
//
// February 2009 by Randall Bohn
// - Added support for writing to EEPROM (what took so long?)
// Windows users should consider WinAVR's avrdude instead of the
// avrdude included with Arduino software.
//
// January 2008 by Randall Bohn
// - Thanks to Amplificar for helping me with the STK500 protocol
// - The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
// - The SPI functions herein were developed for the AVR910_ARD programmer
// - More information at http://code.google.com/p/mega-isp

// #define ARDUINOISP 1

#include <Arduino.h>
#include <HardwareSerial.h>

#include <RFduinoGZLL.h>
#include <SimpleFIFO.h>

#define MAX_BUF 255
#define TX_BUF_MAX 8
SimpleFIFO<uint8_t, MAX_BUF> radioInFifo;
SimpleFIFO<uint8_t, MAX_BUF> radioOutFifo;
uint8_t radioTxBuf[TX_BUF_MAX];          // GZLL radio packet llimit

device_t role = HOST;  // This is the DEVICE code

volatile bool doLowReset = false;
volatile bool doHighReset = false;
volatile bool isPiggyBacking = false;



//#define DEBUG_LVL 1

#ifdef DEBUG_LVL
#ifdef __RFduino__
#define ERROR(x) Serial.println("ERR:"x);
#define DEBUG(x) Serial.println("INF:"x);
#define DEBUGC(x,y) Serial.println(x,y);

#else
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 4); // RX, TX
#define ERROR(x) mySerial.println(x);
#define DEBUG(x) mySerial.println(x);
#define DEBUGC(x,y) mySerial.println(x,y);
#endif

#else
#define ERROR(x)
#define DEBUG(x)
#define DEBUGC(x,y)
#endif

#include "pins_arduino.h"


//#define WRITE_OUT Serial.print
void WRITE_OUT(char* str) {
  int l = strlen(str);

  for (int i = 0; i < l; i++) {
    WRITE_OUT(str[i]);
  }
}
void WRITE_OUT(char c) {
  // /try/ to avoid a race condition!
  // what are the proper mutextes we can use on the ARM RFduiono!!!

  radioOutFifo.enqueue(c);
}


#ifdef __RFduino__


#include <SPI.h>
#define RESET     6
#define LED_HB    2
#define LED_ERR   2
#define LED_PMODE 2
#define PROG_FLICKER true
#else
#define RESET     10
#define LED_HB    13
#define LED_ERR   8
#define LED_PMODE 7
#define PROG_FLICKER true
#endif


#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...

void pulse(int pin, int times);

void setup() {
  Serial.begin(9600);
  pinMode(LED_PMODE, OUTPUT);
  pulse(LED_PMODE, 2);
  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);
  pinMode(LED_HB, OUTPUT);
  pulse(LED_HB, 2);
#ifdef DEBUG_LVL
#ifdef __RFduino__
  Serial.begin(9600);
#else
  mySerial.begin(9600);
#endif
  DEBUG("!!!");
#endif
  rfd_setup();
}

int error = 0;
int pmode = 0;
// address for reading and writing, set by 'U' command
int here;
uint8_t buff[1024]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  int flashpoll;
  int eeprompoll;
  int pagesize;
  int eepromsize;
  int flashsize;
}
parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(LED_HB, hbval);
  delay(20);
}



void loop(void) {
  // is pmode active?
  if (pmode) digitalWrite(LED_PMODE, HIGH);
  else digitalWrite(LED_PMODE, LOW);
  // is there an error?
  if (error) digitalWrite(LED_ERR, HIGH);
  else digitalWrite(LED_ERR, LOW);

  // light the heartbeat LED
  heartbeat();
  rfd_loop();
  if (radioInFifo.count() > 0) {
    avrisp();
  }
}

uint8_t getch() {
  while (radioInFifo.count() == 0) {
    //DEBUG(".");
    //delay(5);
  }
  uint8_t ch = radioInFifo.dequeue();

  //DEBUG("getch:");
  //DEBUGC(ch, HEX);
  return ch;
}
void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();

  }
  DEBUG("done fill");
}

#define PTIME 30
void pulse(int pin, int times) {
  do {
    digitalWrite(pin, HIGH);
    delay(PTIME);
    digitalWrite(pin, LOW);
    delay(PTIME);
  }
  while (times--);
}

void prog_lamp(int state) {
  if (PROG_FLICKER)
    digitalWrite(LED_PMODE, state);
}


void spi_init() {
#ifdef __RFduino__
  SPI.begin();
  SPI.setFrequency(125);

#else
  uint8_t x;
  SPCR = 0x53;
  x = SPSR;
  x = SPDR;
#endif
}

void spi_wait() {
#ifdef __RFduino__
#else
  do {
  }
  while (!(SPSR & (1 << SPIF)));
#endif
}

uint8_t spi_send(uint8_t b) {
#ifdef __RFduino__
  return SPI.transfer (b);

#else
  uint8_t reply;
  SPDR = b;
  spi_wait();
  reply = SPDR;
  return reply;
#endif
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint8_t n;
  spi_send(a);
  n = spi_send(b);
  //if (n != a) error = -1;
  n = spi_send(c);
  return spi_send(d);
}




void empty_reply() {
  if (CRC_EOP == getch()) {
    WRITE_OUT((char)STK_INSYNC);
    WRITE_OUT((char)STK_OK);
  }
  else {
    error++;
    ERROR("empty_reply");
    WRITE_OUT((char)STK_NOSYNC);
  }
}

void breply(uint8_t b) {
  if (CRC_EOP == getch()) {
    WRITE_OUT((char)STK_INSYNC);
    WRITE_OUT((char)b);
    WRITE_OUT((char)STK_OK);
  }
  else {
    error++;
    ERROR("breply");
    WRITE_OUT((char)STK_NOSYNC);
  }
}

void get_version(uint8_t c) {
  switch (c) {
    case 0x80:
      breply(HWVER);
      break;
    case 0x81:
      breply(SWMAJ);
      break;
    case 0x82:
      breply(SWMIN);
      break;
    case 0x93:
      breply('S'); // serial programmer
      break;
    default:
      breply(0);
  }
}

void set_parameters() {
  // call this after reading paramter packet into buff[]
  param.devicecode = buff[0];
  param.revision   = buff[1];
  param.progtype   = buff[2];
  param.parmode    = buff[3];
  param.polling    = buff[4];
  param.selftimed  = buff[5];
  param.lockbytes  = buff[6];
  param.fusebytes  = buff[7];
  param.flashpoll  = buff[8];
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buff[10]);
  param.pagesize   = beget16(&buff[12]);
  param.eepromsize = beget16(&buff[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buff[16] * 0x01000000
                    + buff[17] * 0x00010000
                    + buff[18] * 0x00000100
                    + buff[19];

}

void start_pmode() {
  DEBUG("start_pmode");
  spi_init();
  // following delays may not work on all targets...
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  delay(50);
  digitalWrite(RESET, LOW);
  delay(50);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  pmode = 1;
}

void end_pmode() {
  DEBUG("end_pmode");
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(RESET, INPUT);
  pmode = 0;
}

void universal() {
  int w;
  uint8_t ch;

  fill(4);
  ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
  breply(ch);
}

void flash(uint8_t hilo, int addr, uint8_t data) {
  spi_transaction(0x40 + 8 * hilo,
                  addr >> 8 & 0xFF,
                  addr & 0xFF,
                  data);
}
void commit(int addr) {
  if (PROG_FLICKER) prog_lamp(LOW);
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  if (PROG_FLICKER) {
    delay(PTIME);
    prog_lamp(HIGH);
  }
}

//#define _current_page(x) (here & 0xFFFFE0)
int current_page(int addr) {
  if (param.pagesize == 32)  return here & 0xFFFFFFF0;
  if (param.pagesize == 64)  return here & 0xFFFFFFE0;
  if (param.pagesize == 128) return here & 0xFFFFFFC0;
  if (param.pagesize == 256) return here & 0xFFFFFF80;
  return here;
}


void write_flash(int length) {
  fill(length);
  DEBUG("Fill done");

  if (CRC_EOP == getch()) {
    WRITE_OUT((char) STK_INSYNC);
    WRITE_OUT((char) write_flash_pages(length));
  }
  else {
    error++;
    WRITE_OUT((char) STK_NOSYNC);
    ERROR("write_flash");
  }
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  int page = current_page(here);
  while (x < length) {
    if (page != current_page(here)) {
      commit(page);
      page = current_page(here);
    }
    flash(LOW, here, buff[x++]);
    flash(HIGH, here, buff[x++]);
    here++;
  }

  commit(page);

  return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(int length) {
  // here is a word address, get the byte address
  int start = here * 2;
  int remaining = length;
  if (length > param.eepromsize) {
    error++;
    ERROR("write_eeprom");

    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(int start, int length) {
  // this writes byte-by-byte,
  // page writing may be faster (4 bytes at a time)
  fill(length);
  prog_lamp(LOW);
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
    delay(45);
  }
  prog_lamp(HIGH);
  return STK_OK;
}

void program_page() {
  DEBUG("program_page");

  char result = (char) STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    DEBUG("Write Flash, length:");
    DEBUGC(length, DEC);

    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    DEBUG("Write eeprom");

    result = (char)write_eeprom(length);
    if (CRC_EOP == getch()) {
      WRITE_OUT((char) STK_INSYNC);
      WRITE_OUT(result);
    }
    else {
      error++;
      ERROR("program_page");
      WRITE_OUT((char) STK_NOSYNC);

    }
    return;
  }
  WRITE_OUT((char)STK_FAILED);
  return;
}

uint8_t flash_read(uint8_t hilo, int addr) {
  return spi_transaction(0x20 + hilo * 8,
                         (addr >> 8) & 0xFF,
                         addr & 0xFF,
                         0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x += 2) {
    uint8_t low = flash_read(LOW, here);
    WRITE_OUT((char) low);
    uint8_t high = flash_read(HIGH, here);
    WRITE_OUT((char) high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    WRITE_OUT((char) ee);
  }
  return STK_OK;
}

void read_page() {
  char result = (char)STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  if (CRC_EOP != getch()) {
    ERROR("read_page");

    error++;
    WRITE_OUT((char) STK_NOSYNC);
    return;
  }
  WRITE_OUT((char) STK_INSYNC);
  if (memtype == 'F') result = flash_read_page(length);
  if (memtype == 'E') result = eeprom_read_page(length);
  WRITE_OUT(result);
  return;
}

void read_signature() {
  DEBUG("read_signature");

  if (CRC_EOP != getch()) {
    ERROR("read_signature");

    error++;
    WRITE_OUT((char) STK_NOSYNC);
    return;
  }
  WRITE_OUT((char) STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  WRITE_OUT((char) high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  WRITE_OUT((char) middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  WRITE_OUT((char) low);
  WRITE_OUT((char) STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////


////////////////////////////////////
////////////////////////////////////
int avrisp() {
  uint8_t data, low, high;
  uint8_t ch = getch();

  switch (ch) {
    case '0': // signon
      error = 0;
      empty_reply();
      break;
    case '1':
      if (getch() == CRC_EOP) {
        WRITE_OUT((char) STK_INSYNC);
        WRITE_OUT("AVR ISP");
        WRITE_OUT((char) STK_OK);
      }
      break;
    case 'A':
      get_version(getch());
      break;
    case 'B':
      fill(20);
      set_parameters();
      empty_reply();
      break;
    case 'E': // extended parameters - ignore for now
      fill(5);
      empty_reply();
      break;

    case 'P':
      start_pmode();
      empty_reply();
      break;
    case 'U': // set address (word)
      here = getch();
      here += 256 * getch();
      empty_reply();
      break;

    case 0x60: //STK_PROG_FLASH
      low = getch();
      high = getch();
      empty_reply();
      break;
    case 0x61: //STK_PROG_DATA
      data = getch();
      empty_reply();
      break;

    case 0x64: //STK_PROG_PAGE
      program_page();
      break;

    case 0x74: //STK_READ_PAGE 't'
      read_page();
      break;

    case 'V': //0x56
      universal();
      break;
    case 'Q': //0x51
      error = 0;
      end_pmode();
      empty_reply();
      break;

    case 0x75: //STK_READ_SIGN 'u'
      read_signature();
      break;

      // expecting a command, not CRC_EOP
      // this is how we can get back in sync
    case CRC_EOP:
      ERROR("avrispCRC_EOP");

      error++;
      WRITE_OUT((char) STK_NOSYNC);
      break;

      // anything else we will return STK_UNKNOWN
    default:
      ERROR("avrispDEFAULT");

      error++;
      if (CRC_EOP == getch())
        WRITE_OUT((char)STK_UNKNOWN);
      else
        WRITE_OUT((char)STK_NOSYNC);
  }
}


//////////////////////////


// in RFduino lib v2.0.3 the datarate is 2Mbit

void rfd_setup() {
  RFduinoGZLL.begin(role);  // start the GZLL stack
  pinMode(RESET, OUTPUT);     // set direction of GPIO6
  digitalWrite(RESET, HIGH);  // take Arduino out of reset
}

void rfd_loop() {
  // HOST can't intiate transfer, handled using a 'piggyback' in onReceive callback
  if (doLowReset) {
    digitalWrite(RESET, LOW);
    doLowReset = false;
  }

  if (doHighReset) {
    digitalWrite(RESET, HIGH);
    doHighReset = false;
  }

  static unsigned long lastCall = millis();
  if ( (millis() - lastCall) > 250) {
    if (radioOutFifo.count() != 0 ) {
      DEBUG("Outbuf count:");
      DEBUGC(radioOutFifo.count(), DEC);
    }
    lastCall = millis();
  }

}


void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  if (len > 0) {
    bool skipCommandPacket = false;

    if (4 == len) {
      if ( (data[0] == 'C')  && (data[1] == 'B') && (data[2] == 'R') ) {
        skipCommandPacket = true;
        doLowReset  = (data[3] == 0);
        doHighReset = (data[3] == 1);
      }
    }

    if (!skipCommandPacket) {
      for (int i = 0; i < len ; i++) {
        radioInFifo.enqueue(data[i]);
      }
    }

  }// else {

  // send any pending data on the GZLL 'piggyback', if not currently doing so.

  if (!isPiggyBacking) {
    isPiggyBacking = true;

    int outLen = min(radioOutFifo.count(), TX_BUF_MAX);

    if (outLen > 0) {
      for (int i = 0; i < outLen ; i++) {
        radioTxBuf[i] = radioOutFifo.dequeue();
      }
      RFduinoGZLL.sendToDevice(device, (char*)radioTxBuf, outLen);
    }
    /*
    if (radioOutFifo.count() > 0) {
      radioTxBuf[0] = radioOutFifo.dequeue();
      RFduinoGZLL.sendToDevice(device, (char*)radioTxBuf, 1);
    }
    */
    isPiggyBacking = false;
  }
  // data not sent this time, should be sent on next next zero-length 'poll' packet
}
