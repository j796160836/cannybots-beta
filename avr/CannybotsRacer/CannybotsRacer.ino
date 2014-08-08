//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Libraries
#include <EEPROMex.h>


#include <Cannybots.h>
#include "CannybotsRacer.h"
Cannybots& cb = Cannybots::getInstance();


//TODO move to Cannybots lib
uint32_t  cb_bot_type = 0xCB1FB075;
uint16_t  cb_version  = LF_MAJOR_VERSION*255 + LF_MINOR_VERSION;
uint32_t  cb_bot_id   = 0xCB1FB075;


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
bool debugEnabled = false;



// Bot Config
#define PID_METHOD_1
#define NUM_MOTORS       2
#define NUM_IR_SENSORS 3

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

bool motorDriverHasMode = false;
uint8_t motorDriverMode = false;

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


// REdbot
/*
#define IR_MAX 100
#define WHITE_THRESHOLD 100

#define IR1 A9
#define IR2 A8
#define IR3 A6

#define pinA1 5
#define pinA2 6
#define pinB1 10
#define pinB2 11
*/


// white bot
//#define IR1 A8
//#define IR2 A9
//#define IR3 A10
// orange bot (Custom PCB)
//#define IR1 A6
//#define IR2 A8
//#define IR3 A11

// small white bot
//#define pinA1 6
//#define pinA2 5
//#define pinB1 4
//#define pinB2 3
//#define pin_MODE 7

// orange bot (Custom PCB)
//#define pinA1 3
//#define pinA2 5
//#define pinB1 6
//#define pinB2 9
//#define pin_MODE 2
//#define BOT_TYPE_CUSTOM_PCB 1
//#define IR_MAX 1000
//#define WHITE_THRESHOLD 700


// Indicator LED
#define STATUS_LED 13


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// App State
// PID

#define pid_t int
#define pid_m 1

pid_t Kp = 0;
pid_t Ki = 0;
pid_t Kd = 0;

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
bool IRonBlack[NUM_IR_SENSORS];

volatile unsigned long lastCommandTime = millis();

unsigned long offTheLineTime = 0;
unsigned long offLineLastTime = millis();
bool resetSpeed = true;


// some counters
volatile unsigned long loopNowTime = millis();
volatile unsigned long loopLastTime = millis();
volatile unsigned long loopDeltaTime = millis();
unsigned long loopcount = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////
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
  loopcount++;
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


// read the IR sensors:
// set limit on reading. The reading can be very high and inaccurate on pitch black
void read_ir_sensors() {

#ifndef USE_ANALOG_LIB
  //ANALOG_READ(IR1); delay(ANALOG_READING_DELAY);
#endif
  IRvals[0] = constrain(analogRead(IR1) - IRbias[0], 0, IR_MAX); //left looking from behind

#ifndef USE_ANALOG_LIB
  //ANALOG_READ(IR2); delay(ANALOG_READING_DELAY);
#endif
  IRvals[1] = constrain(analogRead(IR2) - IRbias[1], 0, IR_MAX); //centre

#ifndef USE_ANALOG_LIB
  //ANALOG_READ(IR3); delay(ANALOG_READING_DELAY);
#endif
  IRvals[2] = constrain(analogRead(IR3) - IRbias[2], 0, IR_MAX); //right

  IRonBlack[0] = IRvals[0] > WHITE_THRESHOLD;
  IRonBlack[1] = IRvals[1] > WHITE_THRESHOLD;
  IRonBlack[2] = IRvals[2] > WHITE_THRESHOLD;
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

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Stored Settings  (EEPROM/Flash)

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Remotely called funcs


void lf_updateMotorSpeeds(int _speedA, int _speedB, int _dummy) {
}

void lf_updateAxis(int xAxis, int yAxis, int _dummy) {  
  xAxisValue = xAxis;  //joy X axis vale  = Direction  -255 to 255
  yAxisValue = yAxis;  //joy y axis vale = Throttle    -255 to 255
}


void lf_updatePID(int _Kp, int _Ki, int _Kd) {
  CB_DBG("PID=%d,%d,%d", _Kp, _Ki, _Kd);
  setPID_P(_Kp);
  setPID_D(_Kd); 
  cb.setConfigParameterValue(&cfg_pid_p, &_Kp);
  cb.setConfigParameterValue(&cfg_pid_d, &_Kd);
}

void lf_updateBias (int b1, int b2, int b3) {
  CB_DBG("Bias=%d,%d,%d", b1, b2, b3);
  IRbias[0] = b1;
  IRbias[1] = b2;
  IRbias[2] = b3;
  // TODO: change to the generic:  cb.setConfigParameterValue(&NV_IRBIAS_1), no need to specify variable address again
  cb.setConfigParameterValue(&cfg_ir_bias_1, &IRbias[0]);
  cb.setConfigParameterValue(&cfg_ir_bias_2, &IRbias[1]);
  cb.setConfigParameterValue(&cfg_ir_bias_3, &IRbias[2]);
}

void lf_updateLineFollowingMode(int _forceManualMode, int _d1, int _d2) {
  CB_DBG("ForceManual=%d", _forceManualMode);
  forceManualMode = _forceManualMode;
}

void lf_emitConfig(int _d1, int _d2, int _d3) {
  cb.callMethod(&RACER_PID, getPID_P(), getPID_I(), getPID_D());
  cb.callMethod(&RACER_IRBIAS, IRbias[0], IRbias[1], IRbias[2]);
}

void lf_emitIRValues(int v1, int v2, int v3) {
  static unsigned long lastCall = millis();
  if (millis() - lastCall > 200) {
    cb.callMethod(&RACER_IRVALS, v1, v2, v3);
    lastCall = millis();
  }
}


void lf_ping(int v1) {
  //CB_DBG("ping", v1)
}



//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots glulogic

void mycannybots_setup() {
  cb.registerHandler(&RACER_CRUISESPEED, lf_updateMotorSpeeds);
  cb.registerHandler(&RACER_LINEFOLLOWING_MODE, lf_updateLineFollowingMode);
  cb.registerHandler(&RACER_PID, lf_updatePID);
  cb.registerHandler(&RACER_IRBIAS, lf_updateBias);
  cb.registerHandler(&RACER_JOYAXIS, lf_updateAxis);
  cb.registerHandler(&RACER_CONFIG, lf_emitConfig);
  cb.registerHandler(&RACER_IRVALS, lf_emitIRValues);
  cb.registerHandler(&RACER_PING, lf_ping);

  cb.setConfigStorage(CFG_ID, CFG_BASE, sizeof(cb_app_config), LF_MAJOR_VERSION, LF_MINOR_VERSION);
  cb.registerConfigParameter(&cfg_version, &cb_bot_type);
  cb.registerConfigParameter(&cfg_version, &cb_version);
  cb.registerConfigParameter(&cfg_bot_id, &cb_bot_id);
  cb.registerConfigParameter(&cfg_battery_hasSense, &hasBattSense);
  cb.registerConfigParameter(&cfg_battery_pin_sense, &BATTERY_PIN);
  cb.registerConfigParameter(&cfg_ir_max, &IR_MAX);
  cb.registerConfigParameter(&cfg_ir_whiteThreshold, &WHITE_THRESHOLD);
  cb.registerConfigParameter(&cfg_ir_pin_1, &IR1);
  cb.registerConfigParameter(&cfg_ir_pin_2, &IR2);
  cb.registerConfigParameter(&cfg_ir_pin_3, &IR3);
  cb.registerConfigParameter(&cfg_ir_bias_1, &IRbias[0]);
  cb.registerConfigParameter(&cfg_ir_bias_2, &IRbias[1]);
  cb.registerConfigParameter(&cfg_ir_bias_3, &IRbias[2]);
  cb.registerConfigParameter(&cfg_motorDriver_type, &motorDriverType);
  cb.registerConfigParameter(&cfg_motorDriver_mode, &motorDriverMode);
  cb.registerConfigParameter(&cfg_motorDriver_maxSpeed, &MOTOR_MAX_SPEED);
  cb.registerConfigParameter(&cfg_motorDriver_hasDriveMode, &motorDriverHasMode);
  cb.registerConfigParameter(&cfg_motorDriver_hasMotorSense, &motorDriverHasSense);
  cb.registerConfigParameter(&cfg_motorA_pin_1, &pinA1);
  cb.registerConfigParameter(&cfg_motorA_pin_2, &pinA2);
  cb.registerConfigParameter(&cfg_motorA_pin_sense, &pinAsense);
  cb.registerConfigParameter(&cfg_motorA_postiveSpeedisFwd, &MOTOR_A_POS_IS_FORWARD);
  cb.registerConfigParameter(&cfg_motorA_id, &motorA_id);
  cb.registerConfigParameter(&cfg_motorB_pin_1, &pinB1);
  cb.registerConfigParameter(&cfg_motorB_pin_2, &pinB2);
  cb.registerConfigParameter(&cfg_motorB_pin_sense, &pinBsense);
  cb.registerConfigParameter(&cfg_motorB_postiveSpeedisFwd, &MOTOR_B_POS_IS_FORWARD);
  cb.registerConfigParameter(&cfg_motorB_id, &motorB_id);
//  cb.registerConfigParameter(&cfg_motor_speedSmoothingDivisions, &MAX_MOTOR_DELTA_DIVISOR);
//  cb.registerConfigParameter(&cfg_motor_speedSmoothingMaxDelta, &MAX_MOTOR_DELTA);
  cb.registerConfigParameter(&cfg_pid_p, &Kp);
  cb.registerConfigParameter(&cfg_pid_i, &Ki);
  cb.registerConfigParameter(&cfg_pid_d, &Kd);
//  cb.registerConfigParameter(&cfg_pid_divisor, &PID_DIV);
  cb.registerConfigParameter(&cfg_joystick_xAxisDeadzone, &XAXIS_DEADZONE);
  cb.registerConfigParameter(&cfg_cruiseSpeed_defaultSpeed, &baseCruiseSpeed);
//  cb.registerConfigParameter(&cfg_cruiseSpeed_manualMaxSpeed, &maxCruiseSpeed);
//  cb.registerConfigParameter(&cfg_offLineMaxTime, &OFF_LINE_MAX_TIME);
//  cb.registerConfigParameter(&cfg_info_printValsInterval, &PRINTVALS_INTERVAL);
//  cb.registerConfigParameter(&cfg_debugFlag, &debugEnabled);
  cb.populateVariablesFromConfig();
  cb.begin();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
/// Testing

void test(int16_t p1, int16_t p2, int16_t p3) {

  switch (p1) {
    case CANNYBOTSRACER_TEST_MOTORS:
      motor(MOTOR_MAX_SPEED, 0);
      delay(500);
      motor(-MOTOR_MAX_SPEED, 0);
      delay(500);
      motor(0, MOTOR_MAX_SPEED);
      delay(500);
      motor(0, -MOTOR_MAX_SPEED);
      delay(500);
      motor(0, 0);
      break;
    default:
      break;
  }
}


// bets

// idetify areas of code with high changle volatility

// document:
//          CB API example
//          ARchitecture
//          calss diagram
//        Areas of work

