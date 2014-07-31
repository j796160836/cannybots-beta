#include <RFduinoGZLL.h>
#include <SimpleFIFO.h>

#if 1
#define ERROR(x) Serial.println(x);
#define DEBUG(x) Serial.println(x);
#define DEBUGC(x,y) Serial.println(x,y);
#else
#define ERROR(x)
#define DEBUG(x)
#define DEBUGC(x,y)
#endif

#define MAX_BUF 512
#define TX_BUF_MAX 2

SimpleFIFO<uint8_t, MAX_BUF> radioInFifo;
SimpleFIFO<uint8_t, MAX_BUF> radioOutFifo;
char radioTxBuf[TX_BUF_MAX];


device_t role = HOST;  // This is the DEVICE code

void setup() {
  Serial.begin(9600);
  RFduinoGZLL.begin(role);  // start the GZLL stack
}

void loop() {
  if (radioInFifo.count()>0) {
     while (radioInFifo.count()>0) {
          Serial.write(radioInFifo.dequeue());
      }
      Serial.flush();
   }
  
}


void serialEvent(){      
   while(Serial.available()){    
      DEBUG("gotLocalSerial");     
      radioOutFifo.enqueue(Serial.read());
   }
}
  


volatile bool isPiggyBacking = false;

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  if (len != 0) {
    for (int i = 0; i < len ; i++) {
      radioInFifo.enqueue(data[i]);
    }
  } else {
    DEBUG("dat0");

    // send any pending data on the GZLL 'piggyback', if not currently doing so.
    if (isPiggyBacking) {
      return;
    } else {
      isPiggyBacking = true;
    }
    int outLen = min(radioOutFifo.count(), TX_BUF_MAX);
    
    if (outLen > 0) {
      DEBUG("sndS");

      for (int i = 0; i < outLen ; i++) {
        radioTxBuf[i] = radioOutFifo.dequeue();
      }
      RFduinoGZLL.sendToDevice(device, (char*)radioTxBuf, outLen);
      DEBUG("sndE");
    }
    isPiggyBacking = false;
  }
}

