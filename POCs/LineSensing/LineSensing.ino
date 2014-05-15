// see: http://www.intorobotics.com/line-following-robot-tutorials-how-to-build-programming-code-and-resources/

// Gemera oinfo:
// see: http://www.ikalogic.com/line-tracking-sensors-and-algorithms/


// Tutorial:

// see:  http://www.instructables.com/id/Color-Detection-Using-RGB-LED/


// Sensor Specs:

// Light sensor

// see: https://www.sparkfun.com/datasheets/Sensors/Imaging/TEMT6000.pdf
// see: http://wiring.org.co/learning/basics/ambientlighttemt6000.html


// RGB LED
// see: https://learn.adafruit.com/adafruit-neopixel-uberguide/overview
// see: https://www.adafruit.com/products/1312
// see: 
// wiring:  Adding a ~470 ohm resistor between your microcontroller's data pin and the data input o

// so true: The most common mistake is connecting to the output end of a strip rather than the input.

#import "CircularBuffer.h"

CircularBuffer<uint8_t, 12> sensorBuf1 = CircularBuffer<uint8_t, 12>();
CircularBuffer<uint8_t, 12> sensorBuf2 = CircularBuffer<uint8_t, 12>();

// NeoPixel

#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN 6
enum {RED_COL, GREEN_COL, BLUE_COL, NEOPIXEL_NUM_COLOURS} COLOURS;

uint32_t colours[NEOPIXEL_NUM_COLOURS];

int sensorValue1;
int sensorValue2;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


int currentColour = BLUE_COL;

void setup()
{
  Serial.begin(9600);  // sets the serial port to 9600
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  colours[RED_COL] = strip.Color(255,0,0);        // RED  
  colours[GREEN_COL] = strip.Color(0,255,0);        // Green
  colours[BLUE_COL] = strip.Color(0,0,255);        // Blue

  setLEDColour(colours[currentColour]);
  setLEDBrightnesss(255);
}

int counter = 1;
int sensorAve=0;

int aveWhite = 170;
int aveRed   = 150;
int aveGreen = 50;
int aveBlue  = 90;
int deviation= 20; // RED = 20,   GREEN = 15

void loop()
{
   if (0 /* all colour scan mode */) {
    counter++;
     if ( (counter % 4) == 0) {
       currentColour = (currentColour+1) % NEOPIXEL_NUM_COLOURS;
       setLEDColour(colours[currentColour]);
      counter=0;
      }
  }
 
  sensorValue1 = analogRead(0);       // read analog input pin 0
  sensorBuf1.push(sensorValue1);
  sensorValue2 = analogRead(1);       // read analog input pin 0
  sensorBuf2.push(sensorValue2);
  sensorAve = (sensorBuf1.average()+sensorBuf1.average())/2;

  Serial.print(sensorAve, DEC);
  Serial.print("\t=\t");
  
  if ( RED_COL == currentColour) {
    if ( abs(aveRed-sensorAve)<deviation) {
      Serial.println("On RED");    
    } else if ( sensorAve<100) {  // easier to discrimitate against another primary colour than it is white
       Serial.println("On Other Col");  
    } else {
       Serial.println("On White");  
    }
  } else   if ( GREEN_COL == currentColour) {
    if ( abs(aveGreen-sensorAve)<deviation) {
      Serial.println("On GREEN");    
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
  
  //Serial.print("Ave Intesity: ");
 
 
 
  //Serial.println(sensorBuf1.average(), DEC);
  //Serial.println(sensorBuf1.average(), DEC);
  //Serial.print(currentColourIndex, DEC); 
  //Serial.print("\t");
  //Serial.print(sensorValue1, DEC);
  //Serial.print("\t");  
  //Serial.println(sensorValue2, DEC);  
  //delay(100);                       
}

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
