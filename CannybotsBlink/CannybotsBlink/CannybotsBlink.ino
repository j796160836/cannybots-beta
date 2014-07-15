#include <Cannybots.h>
#include "CannybotsBlink.h"

Cannybots& cb = Cannybots::getInstance();

int led = 13;

void setup() {
  pinMode(led, OUTPUT);
  cb.registerHandler(LED_STATUS, ledStatus);
  cb.begin();
}

void loop() {
  cb.update();
}

void ledStatus(int status) {
    digitalWrite(led, status?HIGH:LOW);   
}
