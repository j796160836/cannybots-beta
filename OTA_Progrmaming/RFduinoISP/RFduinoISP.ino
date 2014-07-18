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
#include <SPI.h>
#include <RFduinoGZLL.h>
#include <SimpleFIFO.h>

SimpleFIFO<byte,255> radioInFifo;
SimpleFIFO<byte,255> radioOutFifo;

#include "pins_arduino.h"
#define RESET     SS

#define LED_HB    9
#define LED_ERR   8
#define LED_PMODE 7
#define PROG_FLICKER true

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

//#define WRITE_OUT Serial.print
#define WRITE_OUT(c) radioInFifo.enqueue(c);

void pulse(int pin, int times);

void setup() {
  Serial.begin(19200);
  pinMode(LED_PMODE, OUTPUT);
  pulse(LED_PMODE, 2);
  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);
  pinMode(LED_HB, OUTPUT);
  pulse(LED_HB, 2);
  
  rfd_setup();
}

int error = 0;
int pmode = 0;
// address for reading and writing, set by 'U' command
int here;
uint8_t buff[256]; // global block storage

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
  if (Serial.available()) {
    avrisp();
  }
  
  rfd_loop();
}

uint8_t getch() {
 // #error read for character received from radio
// add a fifo and return 1 char at a time.
  //while (!Serial.available());
  //return Serial.read();
  while (radioInFifo.count() == 0);
  return radioInFifo.dequeue();
}
void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
  }
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
  uint8_t x;
  SPI.setFrequency(125);
  //SPCR = 0x53;
  //x = SPSR;
  //x = SPDR;
}

void spi_waitOLD() {
  // wait for SPI to be ready.
/*  do {
  }
  while (!(SPSR & (1 << SPIF)));*/
}

uint8_t spi_send(uint8_t b) {
  uint8_t reply;
//  SPDR = b;
//  spi_wait();
//  reply = SPDR;
  reply = SPI.transfer (b);
  return reply;
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
  if (CRC_EOP == getch()) {
    WRITE_OUT((char) STK_INSYNC);
    WRITE_OUT((char) write_flash_pages(length));
  }
  else {
    error++;
    WRITE_OUT((char) STK_NOSYNC);
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
  char result = (char) STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    result = (char)write_eeprom(length);
    if (CRC_EOP == getch()) {
      WRITE_OUT((char) STK_INSYNC);
      WRITE_OUT(result);
    }
    else {
      error++;
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
  if (CRC_EOP != getch()) {
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
      error++;
      WRITE_OUT((char) STK_NOSYNC);
      break;

      // anything else we will return STK_UNKNOWN
    default:
      error++;
      if (CRC_EOP == getch())
        WRITE_OUT((char)STK_UNKNOWN);
      else
        WRITE_OUT((char)STK_NOSYNC);
  }
}






/*
This sketch sets us a DEVICE device to demonstrate 
a serial link between RFDuino modules

This test behaves as a serial pass thru between two RFduinos,
and will reset a target Arduino UNO with GPIO6 for over-air programming

This code uses buffering Serial and Radio packets
Also uses using timout to finding end of serial data

Made by Joel Murphy, Summer 2014
Free to use and share. This code presented as-is. No promises!

*/

#include <RFduinoGZLL.h>

device_t role = DEVICE0;  // This is the DEVICE code

const int numBuffers = 10;              // buffer depth
char serialBuffer[numBuffers] [32];  	// buffers to hold serial data
int bufferLevel = 0;                 	// counts which buffer array we are using
int serialIndex[numBuffers];         	// Buffer position counter
int numPackets = 0;                  	// number of packets to send/receive on radio
int serialBuffCounter = 0;
unsigned long serialTimer;              // used to time end of serial message

char radioBuffer[300];		        // buffer to hold radio data
int radioIndex = 0;                  	// used in sendToHost to protect len value
int packetCount = 0;                    // used to keep track of packets in received radio message
int packetsReceived = 0;                // used to count incoming packets

boolean serialToSend = false;     // set when serial data is ready to go to radio
boolean radioToSend = false;      // set when radio data is ready to go to serial
boolean serialTiming = false;     // used to time end of serial message

unsigned long lastPoll;         // used to time null message to host
unsigned int pollTime = 100;    // time between polls when idling

int resetPin = 6;               // GPIO6 is connected to Arduino UNO pin with 1uF cap in series
boolean toggleReset = false;    // reset flat


void rfd_setup(){
  
  RFduinoGZLL.begin(role);  // start the GZLL stack
  //Serial.begin(57600);     // start the serial port
  
  serialIndex[0] = 1;          // save buffer[0][0] to hold number of packets!
  for(int i=1; i<numBuffers; i++){
    serialIndex[i] = 0;        // initialize indexes to 0
  }
  

  lastPoll = millis();    // set time to perfom next poll 
  
  pinMode(resetPin,OUTPUT);      // set direction of GPIO6
  digitalWrite(resetPin,HIGH);   // take Arduino out of reset
  
  
}



void rfd_loop(){
  
  if(serialTiming){                      // if the serial port is active
    if(millis() - serialTimer > 2){      // if the time is up
      if(serialIndex[bufferLevel] == 0){bufferLevel--;}  // don't send more buffers than we have!
      serialBuffer[0][0] = bufferLevel +1;	// drop the number of packets into zero position
      serialBuffCounter = 0;          		// keep track of how many buffers we send
      serialTiming = false;	       // clear serialTiming flag
      serialToSend = true;             // set serialToSend flag
      lastPoll = millis();             // put off sending a scheduled poll
      RFduinoGZLL.sendToHost(NULL,0);  // send a poll right now to get the ack back
    }
  }

    
  if (millis() - lastPoll > 50){  // make sure to ping the host if they want to send packet
    if(!serialTiming && !serialToSend){  // don't poll if we are doing something important!
      RFduinoGZLL.sendToHost(NULL,0);
    }
    lastPoll = millis();          // set timer for next poll time
  }
  

if(radioToSend){                   // when data comes in on the radio
  for(int i=0; i<radioIndex; i++){
    //Serial.write(();  // send it out the serial port
    radioInFifo.enqueue(radioInFifo);
  }
  radioIndex = 0;                  // reset radioInex counter
  radioToSend = false;             // reset radioToSend flag
}



/*
  dont think we need this, the 
#error WRITE_OUT need to add to a nw internal buffer and set serialToSend=true and add data to     serialBuffer[bufferLevel][serialIndex[bufferLevel]] = Serial.read();    

if(Serial.available()){
  while(Serial.available() > 0){          // while the serial is active
    serialBuffer[bufferLevel][serialIndex[bufferLevel]] = Serial.read();    
    serialIndex[bufferLevel]++;           // count up the buffer size
    if(serialIndex[bufferLevel] == 32){	  // when the buffer is full,
      bufferLevel++;			  // next buffer please
    }  // if we just got the last byte, and advanced the bufferLevel, the serialTimeout will catch it
  }
  serialTiming = true;                   // set serialTiming flag
  serialTimer = millis();                // start time-out clock
}
*/
}// end of loop


void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  
  if(serialToSend){	// send buffer to host during onReceive so as not to clog the radio
    RFduinoGZLL.sendToHost(serialBuffer[serialBuffCounter], serialIndex[serialBuffCounter]);
    serialBuffCounter++;	      // get ready for next buffered packet
    if(serialBuffCounter == bufferLevel +1){// when we send all the packets
      serialToSend = false; 		    // put down bufferToSend flag
      bufferLevel = 0;			    // initialize bufferLevel
      serialIndex[0] = 1;		    // leave room for packet count
      for(int i=1; i<numBuffers; i++){	
        serialIndex[i] = 0;		    // initialize serialInexes
      }
    }
  }
  
  
  if(len > 0){
    int startIndex = 0;	                // get ready to read this packet from 0
    if(packetCount == 0){	        // if this first packet in transaction  
      if(testFirstByte(data[0])){       // if we get a '$' or '#' 
        char dummy[32];                 // set up a trash can
        for(int i=0; i<len; i++){
          dummy[i] = data[i];           // put everything in the trash
        }                               
        return;                         // get outa here!
      }                         // if we didn't get a '$' or '#' the first byte = number of packets 
      packetCount = data[0];	// get the number of packets to expect in message
      startIndex = 1;		// skip the first byte when retrieving radio data
    }		
    for(int i = startIndex; i < len; i++){
      radioBuffer[radioIndex] = data[i];  // read packet into radioBuffer
      radioIndex++;                       // increment the radioBuffer index counter
    }
    packetsReceived++;                    // increment the packet counter
    if(packetsReceived == packetCount){   // when we get all the packets
      packetsReceived = 0;                // reset packets Received for next time
      packetCount = 0;                    // reset packetCount for next time
      radioToSend = true;	          // set radioToSend flag
    }else{                                // if we're still expecting packets,
      RFduinoGZLL.sendToHost(NULL,0);     // poll host for next packet
    }
  }
   
    lastPoll = millis();  // whenever we get an ACK, reset the lastPoll time 

}
  
  
  
boolean testFirstByte(char z){  // test the first byte of a new radio packet for reset msg
  boolean r;
  switch(z){
    case '$':                 // HOST sends '$' when its GPIO6 goes LOW
      digitalWrite(resetPin,LOW);  // clear RESET pin
      r = true;  
      break;
    case '#':                 // HOST sends '#' when its GPIO6 goes HIGH
      digitalWrite(resetPin,HIGH); // set RESET pin
      r = true;
      break;   
    default:
      r = false;
      break;
  }
  return r;
}
  

