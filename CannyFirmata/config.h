

#define DEBUG 1
//#define INFO

#define USE_BLE 1
#define USE_AX12 1
//#define USE_SONAR 1
//#define USE_PIXY
//#define USE_SERVOS
//#define USE_MOTORS
#define USE_IRRECV 1



// Pin Info:

//Genral info:
// Default pijns - BLE = these avialble pins:
// digital 3-8 and analog 0-6
// Pins support PWM  :  3, 5, 6, 9, 10, and 11.  (BLE takes 9,10,11 -   BLE can be configure to use somethng other than 9 & 10)
// Intrurrpt pins:


// so we have digital 3-8 and analog 0-6

// Digital:

// AX12:  0, 1,
// BLE:         2,                , 9, 10, 11, 12, 13
// Servo:               5, 6
// Sonar:         3, 4,       7, 8
// Motor: ???
// IR   : ??
// Free:    None

// Note: the NewPing library used for sonar has a 1-pin mode, need to look at that!
// or try Sonar on 2 analog pins keeping pwm for Moros



// Analog
//
// I2C:             4, 5, 

// FREE:    1, 2, 3,       6


// Motor: where to put up to 4 pins for 2 motors...   4 pins are dir, on/off, brake & sense. could get away wiht just dir and power. 


// BLE Settings

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

// AX12 Settings

#define AX12_MAX_SERVOS 8

// Servo settings
#define SERVO_MAX_SERVOS 2

// SONAR Settings

// Sonar Pinger Settings
#define TRIGGER_PIN1  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1     7  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


// IR  settings

#ifdef USE_IRRECV
#define IRRECV_RECV_PIN 6
#ifdef USE_SERVOS
#error check pin assigments between IR and Servos, this might be an out of date assumption...
#endif
#endif

#define ANALOG_REPORT_NUM 2
#define DIGITAL_REPORT_NUM 2
#define AX12_REPORT_NUM 2


// Data feedback rates
#define MINIMUM_SAMPLE_DELAY 10 
#define ANALOG_SAMPLE_DELAY 0


#ifdef USE_BLE
#include <Adafruit_BLE_UART.h>
extern Adafruit_BLE_UART BLESerial;
#endif

