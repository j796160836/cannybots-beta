#include <EEPROM.h>
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




/////////////////////
bool ledIsOn = false;

void loop_with_publish() {
  cb.update();
  
  //user millis() to calculate on or off ever 5 seconds, assing to status on change
  unsigned long toggleTime = millis() ;
  if ( (millis() - toggleTime) > 5000 ) {
      digitalWrite(led, ledIsOn);   
      ledIsOn = ledIsOn==HIGH ? LOW:HIGH; // flip on or off
      cb.callMethod(LED_STATUS, ledIsOn);
  } 
}










/* 

iOS:

#import "CannybotsController.h"
#import "CannybotsRacer.h"
 
- (void) viewDidAppear:(BOOL)animated {
  
  CannybotsController* cb = [CannybotsController sharedInstance];
  [cb registerHandler:LED_STATUS withBlockFor_INT16_1: ^(int16_t p1)
  {
    [self.onOffSegment setSelectedSegmentIndex:p1];
  }];  
}

- (void) viewWillDisappear:(BOOL)animated {
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb deregisterHandler:RACER_PID];
}    
*/
