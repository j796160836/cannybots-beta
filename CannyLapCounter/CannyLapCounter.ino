#include <RFduinoBLE.h>
#include <RingBuffer.h>

#define LAP_PIN     4
#define TP1 2      //Trig_pin
#define EP1 3      //Echo_pin

#define  TRAINING_SETTLE_TIME 5*1000
#define DEBOUNCE_TIME 1000

char distance_text[128]= {0};


void setup() {
  Serial.begin (9600);
  pinMode(TP1,OUTPUT);      
  pinMode(EP1,INPUT);   
  pinMode(LAP_PIN,OUTPUT);   
  digitalWrite(LAP_PIN,LOW);
  RFduinoBLE.begin();
}

void loop()
{
  long distance = getDistance();
  static int trainStartTime = millis();
  static int trainedAverage = distance;
  static int debounceTime = 0;
  unsigned long timeNow = millis();
  
  if (timeNow - trainStartTime < TRAINING_SETTLE_TIME ) {
    trainedAverage =  (trainedAverage * .8)  + (distance*.2);
  } else {
     // is something close?
     if (distance< trainedAverage/2) {
       // are we still handling the last lap?
         if (timeNow > debounceTime) {
            debounceTime = timeNow + DEBOUNCE_TIME;
            // LAP!!!
            digitalWrite(LAP_PIN, HIGH);
         }
     } else {
        // nothing there
        if (timeNow > debounceTime) {
          digitalWrite(LAP_PIN,LOW);
          // send LAP update
        }
     }
  }
  sprintf(distance_text, "%d,r=%03lu,a=%03lu, dbt=%ul", millis(), distance, trainedAverage, debounceTime);
  Serial.println(distance_text);
}



//get the distance data from HCS-RO4 and convert to a value 
long getDistance()
{       
  // enter a timing critical section
  while (!RFduinoBLE.radioActive);  
  while (RFduinoBLE.radioActive);  
  // 
  digitalWrite(TP1, LOW);                    
  delayMicroseconds(2);
  digitalWrite(TP1, HIGH);                 // pull the Trig pin to high level for more than 10us impulse 
  delayMicroseconds(10);
  digitalWrite(TP1, LOW);
  long microseconds = pulseIn(EP1,HIGH);  
  long distance1 = microseconds/29 / 2; // speed of sound inair, 2 trips

  return distance1;                  
}


