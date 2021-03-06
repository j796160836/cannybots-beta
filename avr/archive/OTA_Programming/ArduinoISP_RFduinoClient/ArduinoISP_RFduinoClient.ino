#include <RFduinoGZLL.h>
#include <SimpleFIFO.h>
/*
Part 1:  Add Programmer

edit:
Applications/Arduino-1.5.6r2.app/Contents/Resources/Java/hardware/arduino/avr/programmers.txt

append:

rfduinoisp.name=RFduino as ISP (Cannybots)
rfduinoisp.communication=serial
rfduinoisp.protocol=avrisp
rfduinoisp.speed=9600
rfduinoisp.program.protocol=avrisp
rfduinoisp.program.speed=9600
rfduinoisp.program.tool=avrdude
rfduinoisp.program.extra_params=-P{serial.port} -b{program.speed}


part 2:  Uploading in IDE


To upload using the programmer either

1. press the 'shift' key when pressing th e 'Upload' button.
The help text will change from 'Upload' to 'Upload Using Programmer'
or
2. Hold down shift while doing the standard upload keyboard shortcut of pressing CMD or ALT and U


Part 3: Burn Bootloader

Just set programmer to RFDuino as ISP and click Burn Bootloader in the Tool menu. No need to hold shift.


Notes:

When uploading the RFduino USB shield Green TX LED will flash whilst uploading
The Red RX LED will flash when verifying.



trouble shooting:

Sympton:

avrdude: Device signature = 0xffffff
avrdude: Yikes!  Invalid device signature.
         Double check connections and try again, or use -F to override


can mean the target has not reset (is wire conneccted? is device on?)
can mean that SPI is not connected correctly (-1 (255) from API

*/
device_t role = DEVICE1;  //  The CannyProxt RFduiono ISP expects DEVICE1 to be a ISP client

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
  if ( (millis() - lastGzllPoll) > 15) {
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

  while (Serial.available() &&  ( radioOutFifo.count() < (MAX_BUF - TX_BUF_MAX))) {
    radioOutFifo.enqueue(Serial.read());
  }

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
  } else {


    static unsigned long lastGzllSend = millis();
    if ( (millis() - lastGzllSend) > 10) {
      //if (serialDataReadyToSend) {
      int outLen = min(radioOutFifo.count(), TX_BUF_MAX);

      if (outLen > 0) {
        for (int i = 0; i < outLen ; i++) {
          char c = radioOutFifo.dequeue();
          radioTxBuf[i] = c;
        }
        RFduinoGZLL.sendToHost(radioTxBuf, outLen);
      }
      //serialDataReadyToSend = false;
      //}
      lastGzllSend = millis();
    }
  }
}


void serialEvent() {

  //serialDataReadyToSend = true;
}


void RFduinoGZLL_onReceive(device_t device, int rssi, char * data, int len) {
  for (int i = 0; i < len ; i++) {
    radioInFifo.enqueue(data[i]);
  }
}
