#include <RFduinoGZLL.h>
#include <SimpleFIFO.h>

device_t role = DEVICE0;  //  DEVICE connects to the PC/Mac



// buffers

#define MAX_BUF 512
#define TX_BUF_MAX 20
SimpleFIFO<char,MAX_BUF> radioInFifo;
SimpleFIFO<char,MAX_BUF> radioOutFifo;
char radioTxBuf[TX_BUF_MAX];       


void setup(){  
  RFduinoGZLL.begin(role);     
  Serial.begin(9600);       
}


volatile unsigned long lastGzllPoll = millis();
void loop() {
  
   if ( (millis() - lastGzllPoll) > 250) {
     RFduinoGZLL.sendToHost(NULL,0);     
     lastGzllPoll = millis();
   }
   
   if (radioInFifo.count()>0) {
     while (radioInFifo.count()>0) {
          Serial.write(radioInFifo.dequeue());
      }
      Serial.flush();
   }
  
  int outLen =min(radioOutFifo.count(),TX_BUF_MAX);    
  
  if (outLen > 0) {      
      for (int i=0; i<outLen ; i++) {
        char c = radioOutFifo.dequeue();
        radioTxBuf[i] = c;
      }
      RFduinoGZLL.sendToHost(radioTxBuf, outLen);
   }
}


void serialEvent(){      
   while(Serial.available()){          
      radioOutFifo.enqueue(Serial.read());
   }
}


void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len){
   for (int i=0; i<len ; i++) {
       radioInFifo.enqueue(data[i]);
   }
}
