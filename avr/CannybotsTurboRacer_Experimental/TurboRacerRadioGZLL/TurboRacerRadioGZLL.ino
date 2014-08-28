//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots Radio Proxy
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version 1.0   -  14.08.2014  -  Inital Version
//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <RFduinoGZLL.h>

#if 1
#define TX_PIN 5
#define RX_PIN 6
#else
#define TX_PIN 1
#define RX_PIN 0
#endif

#define RADIO2SERIAL_MESSAGE_SIZE 22
#define SERIAL2RADIO_MESSAGE_SIZE 20
#define MESSAGE_HEADER_LEN 8      // =   2 (CRC,ID) + 5 (varname) + 1 (bytesleft)


device_t role = HOST;
uint8_t  buffer[RADIO2SERIAL_MESSAGE_SIZE] = {'>', '>'};
volatile static uint8_t  serial2radioBuffer[SERIAL2RADIO_MESSAGE_SIZE] = "HELLO!";

volatile byte sendLen = 0;
volatile bool send = false;
volatile bool sendGzllPiggyBack = false;


void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);        // UART Baud is limited to 9600 when the BLE stack is on.
  //RFduinoGZLL.hostBaseAddress=0x0D0A0704;  // NRF default
  //RFduinoGZLL.deviceBaseAddress0x0E0B0805; // NRF default
  RFduinoGZLL.hostBaseAddress = 0x91827332;
  RFduinoGZLL.begin(role);
}


void loop() {
  if (send) {
    Serial.write(buffer, sendLen);
    //Serial.flush();
    send = false;
  }
  processSerial2Radio();
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  if ( DEVICE2 == device) {
     RFduinoGZLL.sendToDevice(device, (char *)serial2radioBuffer, MESSAGE_HEADER_LEN + serial2radioBuffer[MESSAGE_HEADER_LEN-1]); 
     return; 
  }
  
  if (len < (RADIO2SERIAL_MESSAGE_SIZE - 2)) {
    if (!send) {
      memcpy(buffer + 2, data, len);
      sendLen = len + 2;
      send = true;
    }
  }

  if ( sendGzllPiggyBack) {
    RFduinoGZLL.sendToDevice(device, (char *)serial2radioBuffer, MESSAGE_HEADER_LEN + serial2radioBuffer[MESSAGE_HEADER_LEN-1]); 
    sendGzllPiggyBack=false;
  }
}

void processSerial2Radio() {
  uint8_t msg[SERIAL2RADIO_MESSAGE_SIZE]={0};
  uint8_t msgPtr = 0;  
  static int c = 0, lastChar = 0;
  
  if (Serial.available() >= MESSAGE_HEADER_LEN+1) { 
    lastChar = c;
    c = getCh();
    if ( ( ('>' == c) && ('>' == lastChar) ) ) {
      // read in fixed header
      for (int i = 0 ; i < MESSAGE_HEADER_LEN; i++) {
        msg[msgPtr++] = getCh();
      }
      byte payloadLen     = msg[msgPtr-1];
      byte bytesRemaining = payloadLen;
      //while ( Serial.available() < bytesRemaining);
      
      while (bytesRemaining--) {
        if ( msgPtr < SERIAL2RADIO_MESSAGE_SIZE-MESSAGE_HEADER_LEN) {
          msg[msgPtr++] = getCh();
        } else {
          getCh(); // drop overrun
        }
      }
      if (!sendGzllPiggyBack) {
        memcpy((void*)serial2radioBuffer, msg, msgPtr);
        Serial.write(msg, msgPtr);
        sendGzllPiggyBack=true;
      }
      c = 0;
    }
  }
}



byte getCh() {
  int c = -1;
  while (!Serial.available());
  do {
    c = Serial.read();    
  } while (c == -1);
  return c & 0xFF;
}



