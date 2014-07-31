
/// Config

#define BAUD_RATE 115200

//#define INITIAL_COLOUR RED_COL
//#define INITIAL_COLOUR GREEN_COL
#define INITIAL_COLOUR BLUE_COL
//#define INITIAL_COLOUR WHITE_COL

#define INITIAL_BRIGHTNESS 255


// Filter depth:
//#define LDR_BUFFER_DEPTH 1
#define LDR_BUFFER_DEPTH 10

// Number of LIgth sensors (connected to analog pins 0, 1 , 2, 3, etc)
#define LDR_NUM_SENSORS 1

#define NEOPIXEL_PIN 6


// IGNORE. intesity readings when at 255 brightness using box & breadbord configuration
int aveWhite = 170;
int aveRed   = 150;
int aveGreen = 50;
int aveBlue  = 90;
int deviation= 20; // RED = 20,   GREEN = 15


// PID
unsigned short int Kp = 25 , Ki = 0,Kd = 0;


#if LDR_NUM_SENSORS>1
#define USE_ADC_WORKAROUND_1
//OR
//#define USE_ADC_WORKAROUND_2
#endif

// Misc Line Sensing info:
////////////////////////////////////
// see: http://www.intorobotics.com/line-following-robot-tutorials-how-to-build-programming-code-and-resources/
// see: http://www.ikalogic.com/line-tracking-sensors-and-algorithms/
// see: http://www.instructables.com/id/Color-Detection-Using-RGB-LED/
// see: http://www.societyofrobots.com/member_tutorials/node/361


// Light sensor
////////////////////////////////////
// see: https://www.sparkfun.com/datasheets/Sensors/Imaging/TEMT6000.pdf
// see: http://wiring.org.co/learning/basics/ambientlighttemt6000.html


// RGB LED
////////////////////////////////////
// see: https://learn.adafruit.com/adafruit-neopixel-uberguide/overview
// see: https://www.adafruit.com/products/1312
// wiring:  Adding a ~470 ohm resistor between your microcontroller's data pin and the data input o

// so true: The most common mistake is connecting to the output end of a strip rather than the input.


// Motor Drive (Adafrit)
////////////////////////////////////
// see: https://www.adafruit.com/products/81


// Analog replacement Sensor library
////////////////////////////////////
// see:http://dam.mellis.org/2010/06/sensor_library_for_arduino/

// CHANGES to lib:
// Sensor.cpp:52:      		ADC.setReference(DEFAULT); ->   ADC.setReference(1);



// PID library
//////////////////
// see: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// see: http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/
// see: https://github.com/br3ttb/Arduino-PID-Library/

// PID Autotune lib:
// see: http://playground.arduino.cc/Code/PIDAutotuneLibrary

// Blog on PID and the above library
// see: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

//#include <PID_v1.h>






// Replacement ADC lib
#if defined(USE_ADC_WORKAROUND_2)
#include <Arduino.h>
#undef ADC
#include <ADC.h>
#endif


#import "CircularBuffer.h"


// NeoPixel
#include <Adafruit_NeoPixel.h>
enum {RED_COL, GREEN_COL, BLUE_COL, WHITE_COL, NEOPIXEL_NUM_COLOURS} COLOURS;
uint32_t colours[NEOPIXEL_NUM_COLOURS];
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);



int sensorValues[LDR_NUM_SENSORS];
int sensorAverages[LDR_NUM_SENSORS];
CircularBuffer<uint8_t, LDR_BUFFER_DEPTH>* sensorBuffers[LDR_NUM_SENSORS];



int currentColour = INITIAL_COLOUR;

// Motor driver
#include <AFMotor.h>
AF_DCMotor leftMotor(1);       
AF_DCMotor rightMotor(2);




void setup()
{
  Serial.begin(BAUD_RATE);  


#if defined(USE_ADC_WORKAROUND_2)
  // setup adc
  // ADC prescale suffixes:                 128 ,    64,     32,     16,       8,       4,     and   2
  // providing sample rates of approximately 10 KHz, 19 KHz, 38 KHz, 77 KHz, 153 KHz, 308 KHz, and 615 KHz, respectively.
  ADC.setPrescaleFactor(ADC.PRESCALE128);
#endif

  for (int i =0 ; i< LDR_NUM_SENSORS; i++) {  
    sensorBuffers[i]=new CircularBuffer<uint8_t, LDR_BUFFER_DEPTH>();
  }

  colours[RED_COL] = strip.Color(255,0,0);    
  colours[GREEN_COL] = strip.Color(0,255,0);   
  colours[BLUE_COL] = strip.Color(0,0,255);        
  colours[WHITE_COL] = strip.Color(255,255,255);        

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  setLEDColour(colours[currentColour]);
  setLEDBrightnesss(INITIAL_BRIGHTNESS);
  
  // Motors
  
  leftMotor.setSpeed(255);
  rightMotor.setSpeed(255);
  leftMS();
  rightMS();  
  Serial.println("Setup complete");
}





int counter = 1;
int sensorAve=0;



void loop()
{ 
  updateSensorValues();
  //calcError();
  //moveMotors();
  
  if (0 /* non-0 = all colour scan mode */) {
    counter++;
    if ( (counter % NEOPIXEL_NUM_COLOURS) == 0) {
      currentColour = (currentColour+1) % NEOPIXEL_NUM_COLOURS;
      setLEDColour(colours[currentColour]);
      counter=0;
    }
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LED control
void setLEDColour(uint32_t c) {
   strip.setPixelColor(0, c);
   strip.show();
}

void setLEDColour(uint8_t r,uint8_t g,uint8_t b) {
   strip.setPixelColor(0,  strip.Color(r,g,b));
   strip.show();
}

void setLEDBrightnesss(uint8_t b) {
   strip.setBrightness(b);
   strip.show();
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LDR sensor reading

void updateSensorValues() {
  sensorAve = 0;
  for (int i =0 ; i< LDR_NUM_SENSORS; i++) {    
     
#if defined(USE_ADC_WORKAROUND_1)
    // read with delay because AVR only has 1 ADC internally 
    // see: http://forum.arduino.cc/index.php?topic=87787.0;wap2
    delay(50);
    analogRead(i);       
    delay(50);
    analogRead(i);
#elseif defined(USE_ADC_WORKAROUND_2)
    delayMicroseconds(100);
    //delay(100);
    ADC.setMUX(i);
#endif    

    // smooth sample reading against previous (waaaay too unresponsive)
    //sensorValues[i] = ( 0.9 * sensorValues[i]) + ( 0.1 * analogRead(i));  
    sensorValues[i] = analogRead(i);  
    
    // add value to buffer
    sensorBuffers[i]->push(sensorValues[i]);
    sensorAverages[i] = sensorBuffers[i]->average();
    sensorAve +=  sensorValues[i]; 
  }
  sensorAve = sensorAve/LDR_NUM_SENSORS;
  //Serial.print("All sensorAve=\t");
  //Serial.println(sensorAve, DEC);
  
  Serial.print("Ave Intesity:\t");
  for (int i =0 ; i< LDR_NUM_SENSORS; i++) {    
    Serial.print(sensorAverages[i], DEC);
    Serial.print("\t");
  }
  Serial.println("");
}



bool sensorIsOverTargetColour(byte sensor) {
    if (  abs(20-sensorAverages[sensor]) < 5) {
      return true;
    }
    return false;
}




bool sensorIsOverTargetColourOld(byte sensor) {

  if ( RED_COL == currentColour) {
    if ( abs(aveRed-sensorAve)<deviation) {
      Serial.println("On RED");    
      return true;
    } else if ( sensorAve<100) {  // easier to discrimitate against another primary colour than it is white
       Serial.println("On Other Col");  
       return false;
    } else {
       Serial.println("On White");  
       return false;
    }
  } else   if ( GREEN_COL == currentColour) {
    if ( abs(aveGreen-sensorAve)<deviation) {
      Serial.println("On GREEN");    
      return true;
    } else if ( sensorAve<100) {  // easier to discrimitate against another primary colour than it is white
       Serial.println("On Other Col - NOT GREEN");  
    } else {
       Serial.println("On White");  
    }
  }   if ( BLUE_COL == currentColour) {
    if ( abs(aveBlue-sensorAve)<deviation) {
      Serial.println("On BLUE");    
    } else if ( sensorAve<100) {  // easier to discrimitate against another primary colour than it is white
       Serial.println("On Other Col");  
    } else {
       Serial.println("On White");  
    }
  }
  
  //Serial.print(currentColourIndex, DEC); 
  //Serial.print(sensorValue1, DEC);
  //Serial.print("\t");  
  //Serial.println(sensorValue2, DEC);  
  //delay(100);                       
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Motor control

void leftMF() {
  leftMotor.run(BACKWARD);  
}
void rightMF() {
  rightMotor.run(FORWARD);
}

void leftMB() {
  leftMotor.run(FORWARD);  
}
void leftMotorSpeed(uint8_t s) {
   leftMotor.setSpeed(s);
}
void rightMB() {
  rightMotor.run(BACKWARD);
}

void leftMS() {
  leftMotor.run(RELEASE);  
}
void rightMS() {
  rightMotor.run(RELEASE);
}

void rightMotorSpeed(uint8_t s) {
    rightMotor.setSpeed(s);
}

   


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID

bool pause = true;
char lastreading = 'r';
unsigned int leftpulse, rightpulse,s1, s2,  maxspeed = 250;
float error = 0, perror = 0;
unsigned short int basespeed = 50;
float P, I, D, correction;




// tuning:
// First, set all values to 0 and start with tuning the Kp value. First time I just gave an approximate value. S

// Seeing the robot perform will determine what you should do next. 
//    1) If the robot wobbles a lot reduce the Kp value, 
//    2) if the doesn't follow the line (goes straight in curves) increase the Kp value. 

// Tune the Kp value till the robot smoothly follows the line. 

// By now you will have observed that the robot goes with no acceleration on straight lines. 

// Now, tune the Kd term. After tuning the Kd term move to the Ki term. 

// After which, you will see the robot first center over a straight line and then accelerate also.

// Note: The optimum Kp, Ki and Kd values vary a lot even from track to track. You can only know these optimum values by testing. 



// this assume LDR_NUM_SENSORS  0  is left most
//             LDR_NUM_SENSORS-1 is right most
//             sensors are arranged left to right
void calcError()								//calculates the current error
{				
  s1 = s2 = 0;
  if(sensorIsOverTargetColour(0))		
 {
   lastreading = 'l';
    s1 = 1;	
  }
  if(sensorIsOverTargetColour(LDR_NUM_SENSORS-1)) {
    lastreading = 'r';
    s2 = 1;
  } 
  perror = error;
  //the following statements calculate the error
  error = (s1 * 1) + (s2 * 2);
  error = (error)/(s1+s2);	
  //error = error - 4.5;
}



void moveMotors() {
  if((s1+s2) == 0) {
    //robot has overshot
    
    //checks if the last sensor to the activated was right
    if(lastreading == 'r')
    {
        //turn right at full speed
	rightMB();						
        leftMF();
    }
    //checks if the last sensor to the activated was left
    else if(lastreading == 'l')			
    {
      //turn left at full speed
      rightMB();						
      leftMF();	
    }
  }
 else  									
 {
   //robot on line
   P = error * Kp;
   I += error;
   I = I * Ki;
   D = error - perror;
   correction = P + I + D;
   rightpulse =  basespeed + correction;
   leftpulse = basespeed - correction;				
   leftMF();
   rightMF();
 }
 if(leftpulse > 255)
	leftpulse = 255;
 if(rightpulse > 255)
        rightpulse = 255;	
        
   leftMotorSpeed(leftpulse);
   rightMotorSpeed(rightpulse);
   
}			


