
#include "NT_APP_LineFollowing.h"


#define USE_BLE 1
//#define USE_AX12 1
//#define USE_SONAR 1
//#define USE_PIXY 1
//#define USE_SERVOS
//#define USE_MOTORS
//#define USE_IRRECV 1
//#define USE_TONE
//#define USE_MIC 1


//#define NT_PLATFORM_MBED 1



#ifdef _WIN64
  #define NT_PLATFORM_WIN64 1
#elif _WIN32
  #define NT_PLATFORM_WIN32 1
#elif __APPLE__
    #include "TargetConditionals.h"
    #if TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR
        #define NT_PLATFORM_IOS 1
        #define NT_PLATFORM_IOS_SIM 1
    #elif TARGET_OS_IPHONE
        #define NT_PLATFORM_IOS 1
    #else
        #define NT_PLATFORM_OSX 1
    #endif
#elif __linux
        #define NT_PLATFORM_LINUX 1
#elif __unix // all unices not caught above
        #define NT_PLATFORM_UNIX 1
#elif __posix
        #define NT_PLATFORM_POSIX 1
#else
#define NT_PLATFORM_AVR 1
#endif



//#define DEBUG_LEVEL 1
#define INFO_LEVEL 1


#if defined(NT_PLATFORM_AVR)

#define INFO_OUT_TARGET_UART 1
#define DEBUG_OUT_TARGET_UART 1

//#define INFO_OUT_TARGET_BLE 1
//#define DEBUG_OUT_TARGET_BLE 1

#if defined(INFO_OUT_TARGET_UART)
#define INFO_OUT Serial
#define INFO_OUT_TARGET_UART_BAUD 9600
#endif



#ifdef DEBUG_OUT_TARGET_BLE
// doenst support streams...
#define DBG_OUT BLESerial  
#define DBG(x) BLESerial.println(x);
#else
#define DBG_OUT Serial
#define DBG(x) DBG_OUT << x;
#endif




// Pin Info:

//Genral info:
// Default pins - BLE = these avialble pins:
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
// IR   :                  6!
// Tone:                5!
// Free:    None

// Note: the NewPing library used for sonar has a 1-pin mode, need to look at that!
// or try Sonar on 2 analog pins keeping pwm for Moros



// Analog
//
// I2C*:              4, 5, 
// MIC:    0,
// FREE:      1, 2, 3,      6

// I2C to support: 3/6/9 axis accelraometers, compass, pixy, etc


// Motor: where to put up to 4 pins for 2 motors...   4 pins are dir, on/off, brake & sense. could get away wiht just dir and power. 

// IR wiring:    

// Spkr wiring:   100k resister between 

// Mic wiring:  10k resister between 5v and vcc




// Pixy wiring:
// Dont forget to set the dataout mode in the Pixymon app to 1 (I2C) !!!
// IDC CAble Wiring
// (red) 1 2 3 4 5 6 7 8 9 10
// (red)
//  ___|-|___  
//  1 3 5 7 9
//  2 4 6 8 10
// ----------
// 2 = 5v
// 5 = SCL
// 8 = GND
// 9 = SDA
//  ___|-|___  
//  . . C . D
//  V . . G .
// ----------




// BLE Settings

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

// AX12 Settings

#define AX12_MAX_SERVOS 4

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

// Pixy settings

#define PIXY_CAM_WIDTH 320
#define PIXY_CAM_X_MID 160
#define PIXY_MARGIN 40



// System setting

// Data feedback rates
#define MINIMUM_SAMPLE_DELAY 150 
#define ANALOG_SAMPLE_DELAY 0


#ifdef USE_BLE
#include <Adafruit_BLE_UART.h>
extern Adafruit_BLE_UART BLESerial;
#endif


#ifdef USE_PIXY
#include <Wire.h>
#include <PixyI2C.h>
#endif


#ifdef USE_TONE
#define TONE_PIN 5 
#endif

#ifdef USE_MIC
#define MIC_PIN 0
#endif


#endif

////////////////////////////


