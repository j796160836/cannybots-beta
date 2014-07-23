#include <RFduinoGZLL.h>
#include <SimpleFIFO.h>

device_t role = DEVICE0;  //  DEVICE connects to the PC/Mac

// Pinouts

#define RESET     6

#define SEND_STATUS_LED 2
#define RECV_STATUS_LED 3


// buffers

#define MAX_BUF 255
#define TX_BUF_MAX 30
SimpleFIFO<char, MAX_BUF> radioInFifo;
SimpleFIFO<char, MAX_BUF> radioOutFifo;
char radioTxBuf[TX_BUF_MAX];          // GZLL radio packet llimit


volatile bool serialDataReadyToSend = false;


// state

int resetPin = 6;              // using GPIO6 to feel the DTR go low
int resetPinValue;             // value of GPIO6 goes here
int lastResetPinValue;         // used to watch the rising/falling edge of GPIO6
boolean toggleReset = false;   // reset flag


char resetMessage[4] = {'C', 'B', 'R', 0x00};


void setup() {

  RFduinoGZLL.begin(role);     // start the GZLL stack
  Serial.begin(9600);        // start the serial port

  pinMode(SEND_STATUS_LED, OUTPUT);
  pinMode(RECV_STATUS_LED, OUTPUT);
  digitalWrite(SEND_STATUS_LED, HIGH);
  digitalWrite(RECV_STATUS_LED, HIGH);
  delay(200);
  digitalWrite(SEND_STATUS_LED, LOW);
  digitalWrite(RECV_STATUS_LED, LOW);

  pinMode(resetPin, INPUT);    // DTR routed to GPIO6
  lastResetPinValue = digitalRead(resetPin);  // initialize the lastResetPinValue

}


unsigned long lastGzllPoll = millis();
void pollGZLL() {
    // get and pending data from the HOST
  if ( (millis() - lastGzllPoll) > 25) {
    RFduinoGZLL.sendToHost(NULL, 0);
    lastGzllPoll = millis();
  }
    // process inbound data
  if (radioInFifo.count() > 0) {
    while (radioInFifo.count() > 0) {
      Serial.write(radioInFifo.dequeue());
    }
    Serial.flush();
  }
}


void loop() {

  pollGZLL();

  resetPinValue = digitalRead(resetPin);    // read the state of DTR
  if (resetPinValue != lastResetPinValue) { // if it's changed
    if (resetPinValue == LOW) {
      resetMessage[3] = 0;                // '$' indicates falling edge

      digitalWrite(RECV_STATUS_LED, HIGH);
      delay(50);
      digitalWrite(RECV_STATUS_LED, LOW);

    } else {
      resetMessage[3] = 1;                // '#' indicates rising edge

      digitalWrite(SEND_STATUS_LED, HIGH);
      delay(50);
      digitalWrite(SEND_STATUS_LED, LOW);

    }
    lastResetPinValue = resetPinValue;      // keep track of resetP in state

    RFduinoGZLL.sendToHost(resetMessage, 4);  // send message to DEVICE
  }


  static unsigned long lastGzllSend = millis();
  if ( (millis() - lastGzllSend) > 50) {
    if (serialDataReadyToSend) {
      int outLen = min(radioOutFifo.count(), TX_BUF_MAX);

      if (outLen > 0) {
        for (int i = 0; i < outLen ; i++) {
          char c = radioOutFifo.dequeue();
          radioTxBuf[i] = c;
        }
        RFduinoGZLL.sendToHost(radioTxBuf, outLen);
      }
      serialDataReadyToSend = false;
    }
    lastGzllSend = millis();
  }
}

void serialEvent() {
  while (Serial.available()) {
    radioOutFifo.enqueue(Serial.read());
  }
  serialDataReadyToSend = true;  
}


void RFduinoGZLL_onReceive(device_t device, int rssi, char * data, int len) {
  for (int i = 0; i < len ; i++) {
    radioInFifo.enqueue(data[i]);
  }
}
