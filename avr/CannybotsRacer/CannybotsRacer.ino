//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Libraries
#include <EEPROMex.h>
#include <Cannybots.h>
#include "CannybotsRacer.h"
Cannybots& cb = Cannybots::getInstance();

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Bot Config

// TODO:  to be tidied,  naming convention and moved to a single struct (same as config) (e.g. reduce to a single line here)

#define NUM_MOTORS       2
#define NUM_IR_SENSORS 3
#define STATUS_LED 13
bool debugEnabled = false;
// PID
uint8_t  PID_SAMPLE_TIME      = 5;
uint8_t  PID_DIV              = 10;
uint16_t PRINTVALS_INTERVAL   = 1000;
uint16_t OFF_LINE_MAX_TIME    = 100;
uint16_t MANUAL_MODE_RADIOSILENCE_TIMEOUT = 500;
// Motor settings
bool MOTOR_A_POS_IS_FORWARD = 1;
bool MOTOR_B_POS_IS_FORWARD = 1;
uint8_t motorA_id = 0;
uint8_t motorB_id = 1;
uint8_t motorDriverType = 0;
bool motorDriverHasMode = true;
bool motorDriverHasSense = false;
uint8_t MOTOR_MAX_SPEED          = 255;
uint8_t MAX_MOTOR_DELTA_DIVISOR  =  100.0;                // number of divisions between current and target motor speed
uint8_t MAX_MOTOR_DELTA          =   255.0;              // max motor speed change
uint8_t XAXIS_DEADZONE           = 50;                             // Joystick
// Pinout wiring for Aiva
#define BOT_TYPE_CUSTOM_PCB 1
uint16_t IR_MAX = 1000;
uint16_t WHITE_THRESHOLD = 700;
uint8_t IR1 = A6;
uint8_t IR2 = A8;
uint8_t IR3 = A11;
uint8_t pinA1 = 3;
uint8_t pinA2 = 5;
uint8_t pinB1 = 6;
uint8_t pinB2 = 9;
uint8_t pin_MODE = 2;
uint8_t pinAsense = 0;
uint8_t pinBsense = 0;
bool    hasBattSense =false;
uint8_t BATTERY_PIN=A1;                                  // Battery Voltage sensing


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// App State

// TODO:  KEEP!  IR and AIS here!!!!!


// 'Core' line following
// TODO: to be move inside the 'to be' LineFollowing Library
int IRvals[NUM_IR_SENSORS];
int IRbias[NUM_IR_SENSORS];
// Speed
// TODO: to be move inside the 'to be' LineFollowing Library
int baseCruiseSpeed = 150;
int cruiseSpeed = baseCruiseSpeed;
int speedA = 0;
int speedB = 0;


// Lap Timing

unsigned long currentStartLapTime = 0;
int  lapCount=0;


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// PID

int Kp = 0;
int Ki = 0;
int Kd = 0;
int P_error = 0;
int D_error = 0;
int error = 0;
int error_last = 0; // to calculate D_error = error - error_last
int correction = 0; //error after PID filter

// Joystick
int yAxisValue = 0;  // -255..255
int xAxisValue = 0;  // -255..255


void pid_calculate() {
  // Note: to override config:
  Kp= 3;
  Kd= 1;  
  
  // process IR readings via PID
  error_last = error;                                   // store previous error before new one is caluclated
  error = IRvals[0] - IRvals[2];                        // TODO: change to lineFOllowingLib.getIRreading(IR_SENSORID);

  P_error = error * Kp / PID_DIV;                               // calculate proportional term
  D_error = (error - error_last) * Kd / PID_DIV;                // calculate differential term
  correction = P_error + D_error;
  
  speedA = cruiseSpeed + correction;
  speedB = cruiseSpeed - correction;
}


void  joystick_lineFollowingControlMode() {
    speedA = speedA + (yAxisValue/3); //superpose yAxis with PID output speed
    speedB = speedB + (yAxisValue/3);
}


void joystick_manualControlMode() {
    speedA =  (yAxisValue + xAxisValue)/4; //-xAxisValue
    speedB =  (yAxisValue + xAxisValue)/4; //xAxisValue
}


// Lap Timing

// this should be called at least once to inform the phone that the bot has started racing
void lap_started() {
  currentStartLapTime = millis();
  cb.callMethod(&LAPCOUNTER_GETREADY, currentStartLapTime); 
}

// call on lap complete, updates phone with current lap time, lap count and restarts the lap timer
void lap_completed() {
  cb.callMethod(&LAPCOUNTER_LAPTIME, millis()-currentStartLapTime);
  cb.callMethod(&LAPCOUNTER_LAPCOUNT, lapCount);
  currentStartLapTime = millis();
  lapCount++;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Setup & Main Loop

void setup() {
  lineFollowing_setup();
  cannybots_setup(); 
}


void loop() {
  lineFollowing_loop();
  cb.update();
}

