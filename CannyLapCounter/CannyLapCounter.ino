#include <Ultrasonic.h>

#ifdef __RFduino__
#include <RFduinoBLE.h>
#include <RingBuffer.h>
#endif 

#define BLE_LOCALNAME "CBLapCounter"
#include <Cannybots.h>
#include "CannyLapCounter.h"
Cannybots& cb = Cannybots::getInstance();

#define WATCHDOG_TIMEOUT 5
#ifdef __RFduino__
#define LAP_PIN     4
#else 
#define LAP_PIN     13
#endif

#define TP1 2      //Trig_pin
#define EP1 3      //Echo_pin

#define TRAINING_SETTLE_TIME 2*1000
#define DEBOUNCE_TIME 250

char distance_text[128]= {0};


bool resetLapTimeOnNextTrigger=false;
unsigned long currentStartLapTime = 0;


void getReady(int16_t p1) {
  resetLapTimeOnNextTrigger=true;
  CB_DBG("getReady!",0);
  Keyboard.println("New driver!\n");
}
void lapTime(int16_t p1) {
}


void sendLapTime(unsigned long lapTime) {
  cb.callMethod(LAPCOUNTER_LAPTIME, lapTime);
  Keyboard.print("Laptime: ");
  Keyboard.println(lapTime/1000.0);
}


void setup() {
  Serial.begin (9600);
  CB_DBG("START",0);

  pinMode(TP1,OUTPUT);      
  pinMode(EP1,INPUT);   
  pinMode(LAP_PIN,OUTPUT);   
  //digitalWrite(LAP_PIN, HIGH);
  //delay(1000);
  //digitalWrite(LAP_PIN,LOW);
  cb.registerHandler(LAPCOUNTER_GETREADY, getReady);
  cb.registerHandler(LAPCOUNTER_LAPTIME, lapTime);
  cb.begin();
  WATCHDOG_SETUP(WATCHDOG_TIMEOUT);
  CB_DBG("START complete",0);
  
  Keyboard.begin();
}



void loop()
{
  WATCHDOG_RELOAD();
  cb.update();
  
  static unsigned long trainStartTime = millis();
  static int trainedAverage = 100;
  static unsigned long debounceTime = 0;
  static bool trained;
  unsigned long timeNow = millis();
  static bool okToSend = false;
  long distanceReading = getDistance();
  static long distance=0;
  CB_DBG("Dist: %d - %d", distanceReading, distance);
  if (!trained) {
    trainedAverage =  (trainedAverage * .8)  + (distanceReading*.2);
    if ((timeNow - trainStartTime) > (TRAINING_SETTLE_TIME)) {
      trained=true;
    }
  }
 

  if (1) {
     // is something close?
     distance = (distanceReading * .4)  + (distance*.6);

      
     if (distance< trainedAverage/2) {
       // are we still handling the last lap?
         if (timeNow > debounceTime) {
            debounceTime = timeNow + DEBOUNCE_TIME;
            // LAP!!!
            digitalWrite(LAP_PIN, HIGH);
            okToSend = true;
         }
     } else {
        // nothing there
        if (timeNow > debounceTime) {
          digitalWrite(LAP_PIN,LOW);
          // send LAP update
          if (okToSend) {
            // is the next trigger meant to start the clock?
            if (resetLapTimeOnNextTrigger) {
              resetLapTimeOnNextTrigger=false;
              currentStartLapTime = millis();
            } else {
              sendLapTime(millis()-currentStartLapTime);
              currentStartLapTime = millis();
            }
            okToSend= false;
          }

        }
     }
  }
  //CB_DBG("%lu,r=%03lu,a=%03lu, dbt=%lu", millis(), distance, trainedAverage, debounceTime);
  //Serial.println(distance_text);
}

Ultrasonic ultrasonic( TP1, EP1 );
long getDistance() {
   return ultrasonic.Ranging(CM);
}


