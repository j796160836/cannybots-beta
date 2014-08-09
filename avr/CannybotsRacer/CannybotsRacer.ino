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
#define NUM_MOTORS       2
#define NUM_IR_SENSORS 3
#define STATUS_LED 13
//TODO move to Cannybots lib
uint32_t  cb_bot_type = 0xCB1FB075;
uint16_t  cb_version  = LF_MAJOR_VERSION*255 + LF_MINOR_VERSION;
uint32_t  cb_bot_id   = 0x0000CB01; 

bool debugEnabled = false;

// PID
uint8_t  PID_SAMPLE_TIME      = 10;
uint8_t  PID_DIV              = 10;

uint16_t PRINTVALS_INTERVAL   = 1000;
uint16_t OFF_LINE_MAX_TIME    = 200;
uint16_t MANUAL_MODE_RADIOSILENCE_TIMEOUT = 500;

// Motor settings
bool MOTOR_A_POS_IS_FORWARD = 1;
bool MOTOR_B_POS_IS_FORWARD = 0;
uint8_t motorA_id = 0;
uint8_t motorB_id = 1;
uint8_t motorDriverType = 0;
bool motorDriverHasMode = true;
bool motorDriverHasSense = false;
uint8_t MOTOR_MAX_SPEED          = 255;
uint8_t MAX_MOTOR_DELTA_DIVISOR  =  100.0;                // number of divisions between current and target motor speed
uint8_t MAX_MOTOR_DELTA          =   255.0;              // max motor speed change

uint8_t XAXIS_DEADZONE           = 50;                             // Joystick

// Pinout wiring
// Aiva
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
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// App State
// PID

#define pid_t int
#define pid_m 1

pid_t Kp = 5;
pid_t Ki = 0;
pid_t Kd = 3;

pid_t P_error = 0;
pid_t D_error = 0;
pid_t error = 0;
pid_t error_last = 0; // to calculate D_error = error - error_last
pid_t correction = 0; //error after PID filter

/////

bool isLineFollowingMode = true;
bool forceManualMode = false;
int baseCruiseSpeed = 150;
int cruiseSpeed = baseCruiseSpeed;


int speedA = 0;
int speedB = 0;


// Joystick
int yAxisValue = 0;  // -255..255
int xAxisValue = 0;  // -255..255

int IRvals[NUM_IR_SENSORS];
int IRbias[NUM_IR_SENSORS];

unsigned long offTheLineTime = 0;
unsigned long offLineLastTime = millis();


// some counters
volatile unsigned long lastCommandTime = millis();
volatile unsigned long loopNowTime = millis();
volatile unsigned long loopLastTime = millis();
volatile unsigned long loopDeltaTime = millis();

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Setup & Main Loop

void setup() {
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

#ifdef BOT_TYPE_CUSTOM_PCB
  pinMode(pin_MODE, OUTPUT);
  digitalWrite(pin_MODE, HIGH); //to set controller to Phase/Enable mode
#endif

  mycannybots_setup();
}

void loop() {
  // do some stats...  
  loopNowTime = millis();
  loopDeltaTime = loopNowTime - loopLastTime;
  loopLastTime = loopNowTime;
  
  // read IR sensor values
  read_ir_sensors();
  // publish IR values
  lf_emitIRValues(IRvals[0], IRvals[1], IRvals[2]);
  
  // count up the time spent off the line, rahter than switching to manual mode the instance the mid sensor goes of the line
  if ((IRvals[1] <= WHITE_THRESHOLD )) {
    offTheLineTime += loopNowTime - offLineLastTime;
    offLineLastTime = loopNowTime;
    if (offTheLineTime > OFF_LINE_MAX_TIME) {
      isLineFollowingMode = 0;
    }
  } else {
    offTheLineTime = 0;
    isLineFollowingMode = 1;
  }
  
  if (forceManualMode) {
    isLineFollowingMode = 0;
  }

  lf_report_followingMode(isLineFollowingMode);

  if (isLineFollowingMode) {
    calculate_PID();
    speedA = speedA + (yAxisValue/3); //superpose yAxis with PID output speed
    speedB = speedB + (yAxisValue/3);
  } else {
    // in manual mode
    if ( (millis() - cb.getLastInboundCommandTime()) > MANUAL_MODE_RADIOSILENCE_TIMEOUT) {
      // no command has been received in the last X millis, err on the side of caution and stop!
      speedA = speedB =  0;
    } else {
      // just allow revese move to get back to line.. 
      if (yAxisValue < 0){
        speedA = yAxisValue/5; //-xAxisValue
        speedB = yAxisValue/5; //xAxisValue
      } else {
        speedA = speedB = 0;
      }
    }
  }
  motor(speedA, speedB);
  delay(5);
  printvalues();
  cb.update();
}



//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors and actuators

// TODO: move to a CannyBots line following library


// read the IR sensors:
// set limit on reading. The reading can be very high and inaccurate on pitch black
void read_ir_sensors() {
  //analogRead(IR1); delay(ANALOG_READING_DELAY);
  IRvals[0] = constrain(analogRead(IR1) - IRbias[0], 0, IR_MAX); //left looking from behind
  //analogRead(IR2); delay(ANALOG_READING_DELAY);
  IRvals[1] = constrain(analogRead(IR2) - IRbias[1], 0, IR_MAX); //centre
  //analogRead(IR3); delay(ANALOG_READING_DELAY);
  IRvals[2] = constrain(analogRead(IR3) - IRbias[2], 0, IR_MAX); //right
  
  //delay (100);
  //CB_DBG("%d,%d,%d, A=%d,%d,%d, IRB(%d,%d,%d), IRMAX=%d, WTHR=%d", IR1, IR2, IR3, A6, A8, A11, IRbias[0],IRbias[1], IRbias[2], IR_MAX, WHITE_THRESHOLD);
}

void motor(int _speedA, int _speedB) {
  // TODO: read config from eeprom
#ifdef BOT_TYPE_CUSTOM_PCB
  motor_customPCB(_speedA, _speedB);  // TODO:  move to config!!
#else
  motor_ORG(_speedA, _speedB);
#endif

}

void motor_customPCB(int _speedA, int _speedB)
{
  _speedA = constrain(_speedA, -255, 255);
  _speedB = constrain(_speedB, -255, 255);

  digitalWrite(pinA1, _speedA >= 0 ? HIGH : LOW) ;
  analogWrite (pinA2, abs(_speedA));

  digitalWrite(pinB1, _speedB >= 0 ? HIGH : LOW);
  analogWrite (pinB2, abs(_speedB));
}


// motor controller function
void motor_ORG(int _speedA, int _speedB) // V4
{
  _speedA = constrain(_speedA, -255, 255);
  _speedB = constrain(_speedB, -255, 255);
  if (_speedA >= 0) {
    analogWrite(pinA1, _speedA);
    analogWrite(pinA2, 0);
  } else {
    analogWrite(pinA1, 0);
    analogWrite(pinA2, abs(_speedA));
  }
  if (_speedB >= 0) {
    analogWrite(pinB1, _speedB);
    analogWrite(pinB2, 0);
  } else {
    analogWrite(pinB1, 0);
    analogWrite(pinB2, abs(_speedB));
  }
}




//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Utils

bool sign(double x) {
  return ((x > 0) - (x < 0));
}

void printvalues ()
{
  static unsigned long lastPrint = millis();
  if ( millis() - lastPrint < PRINTVALS_INTERVAL )
    return;
  lastPrint = millis();
  printvals_PID();
}

void lf_report_followingMode(bool isLineMode) {
  static unsigned long lastCall = millis();
  // throttle sending to 1000/x times a second
  if (millis() - lastCall > 500) {
    cb.callMethod(&RACER_LINEFOLLOWING_MODE, isLineMode);
    lastCall = millis();
  }
}


