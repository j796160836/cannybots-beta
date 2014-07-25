//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Libraries


#include <SPI.h>
#include <Cannybots.h>
#include "CannybotsRacer.h"
Cannybots& cb = Cannybots::getInstance();


//////////////////////////////////////////////////////////////////////////////////////////////////
//#define USE_ANALOG_LIB
#ifdef USE_ANALOG_LIB
// see: https://github.com/merose/AnalogScanner
#include <AnalogScanner.h>
AnalogScanner scanner;
#define ANALOG_READ scanner.getValue
#else
#define ANALOG_READ analogRead
#endif


bool sign(double x) { return ((x>0)-(x<0)); }


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Config

      
// number of divisions between current and target motor speed            
#define MAX_MOTOR_DELTA_DIVISOR     1.0
// max motor speed change
#define MAX_MOTOR_DELTA             255.0

//#define MOTOR_A_POS_IS_FORWARD 1


// Pinout wiring

// Aiva
#define BOT_TYPE_CUSTOM_PCB 1
#define IR_MAX 1000
#define WHITE_THRESHOLD 700

#define IR1 A6
#define IR2 A8
#define IR3 A11

#define pinA1 3 
#define pinA2 5
#define pinB1 6
#define pinB2 9

#define pin_MODE 2



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

// Joystick

#define XAXIS_DEADZONE 50


// IR Sensor settings

#define NUM_IR_SENSORS   3


// Motor settings
#define NUM_MOTORS       2
#define MOTOR_MAX_SPEED 255

// Battery Voltage sensing
#ifdef BOT_TYPE_CUSTOM_PCB
#define BATTERY_PIN A1
#endif 

// Indicator LED
#define STATUS_LED 13
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// App State

// Pid
int Kp = 0;
int Ki = 0;
int Kd = 0;
int I_limit = 100;
int P_error = 0;
int I_error = 0;
int D_error = 0;
int error = 0;
int error_last = 0; // to calculate D_error = error - error_last
int correction = 0; //error after PID filter

bool isLineFollowingMode = true;
bool forceManualMode = false;
int baseCruiseSpeed = 180;
int cruiseSpeed = baseCruiseSpeed;

int speedA = 0;
int speedB = 0;
int manualA = 0;
int manualB = 0;
//int speeds[NUM_MOTORS];

int yAxisValue = 0;
int xAxisValue = 0;

int IRvals[NUM_IR_SENSORS];
int IRbias[NUM_IR_SENSORS];

volatile unsigned long lastCommandTime = millis();

// NV settings
#include <EEPROM.h>

#define NV_ID                           "CBLF"
#define NV_BASE                         64
#define NV_VERSION                      0
#define NV_PID_P                        1
#define NV_PID_I                        2
#define NV_PID_D                        3
#define NV_PID_ALGO_TYPE                4
#define NV_DEFAULT_CRUISESPEED          5
#define NV_MAX_MANUAL_CRUISESPEED       6
#define NV_MAX_MOTOR_SPEED              7
#define NV_MOTOR_ALGO_TYPE              8
#define NV_M1_A_PIN                     9
#define NV_M1_B_PIN                    10
#define NV_M1_SENSE_PIN                11
#define NV_M2_A_PIN                    12
#define NV_M2_B_PIN                    13
#define NV_M2_SENSE_PIN                14
#define NV_MDRIVER_MODE_PIN            15
#define NV_BAT_PIN                     16
//   bits [0..7] =  [ HAS_MDRRIVEMODE, HAT_MOTOR_SENSE, HAS_BAT_SENSE, M1_POSITIVESPEED_IS_FWD, M2_POSITIVESPEED_IS_FWD, n/a, n/a, n/a],
#define NV_FEATURES_MASK1              17  
#define NV_IR_MAX                      18 // UINT
#define NV_IR_WHITE_THRESHOLD          20 // UINT
#define NV_XAXIS_DEADZONE              22 // BYTE
#define NV_BOT_ID                      23 // UINT
#define NV_MAX_MOTOR_DELTA             24
#define NV_MAX_MOTOR_DELTA_DIV         25
#define NV_IRPIN_1                     30
#define NV_IRPIN_2                     31
#define NV_IRPIN_3                     32
#define NV_IRBIAS_1                    40 // INT8
#define NV_IRBIAS_2                    41 // INT8
#define NV_IRBIAS_3                    42 // INT8

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
#ifdef BOT_TYPE_CUSTOM_PCB 
  pinMode(pin_MODE, OUTPUT);
  digitalWrite(pin_MODE, HIGH); //to set controller to Phase/Enable mode
#endif

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
#ifdef USE_ANALOG_LIB
  int scanOrder[NUM_IR_SENSORS+1] = {IR1, IR2, IR3, BATTERY_PIN};
  scanner.setScanOrder(NUM_IR_SENSORS+1, scanOrder);
  scanner.beginScanning();
  delay(1); // Wait for the first scans to occur.
#endif
  cannybots_setup();
}


void loop() {
  read_ir_sensors();
  lf_emitIRValues(IRvals[0], IRvals[1], IRvals[2]);
  
  isLineFollowingMode =  IRvals[1] >= WHITE_THRESHOLD;
  if (forceManualMode)
    isLineFollowingMode=0;
    
  lf_report_followingMode(isLineFollowingMode);

  if (isLineFollowingMode) {
    calculate_PID();
  } else {
    // in manual mode
    if ( (millis() - cb.getLastInboundCommandTime()) > 2000) {
      // no command has been received in the last 2 seconds, err on the side of caution and stop!
      speedA = speedB = 0;
    } else {
      speedA = speedA + min((manualA-speedA)/MAX_MOTOR_DELTA_DIVISOR, MAX_MOTOR_DELTA*sign(manualA-speedA));
      speedB = speedB + min((manualB-speedB)/MAX_MOTOR_DELTA_DIVISOR, MAX_MOTOR_DELTA*sign(manualB-speedB));
    }
  }
  motor(speedA, speedB);  
  printvalues();

  cb.update();
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Algorithms

#ifdef BOT_TYPE_CUSTOM_PCB
#define PID_DIV 10
#else
#define PID_DIV 1
#endif
void calculate_PID() {
  // process IR readings via PID
    error_last = error;                                   // store previous error before new one is caluclated
    //error = constrain(IRvals[0] - IRvals[2], -30, 30);        // set bounds for error
    error = IRvals[0] - IRvals[2];
    
    P_error = error * Kp / PID_DIV;                               // calculate proportional term
    D_error = (error - error_last) * Kd / PID_DIV;                // calculate differential term
    correction = P_error + D_error;
    cruiseSpeed = baseCruiseSpeed + manualA;
#ifdef MOTOR_A_POS_IS_FORWARD
    speedA = constrain(cruiseSpeed - correction, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    speedB = constrain(cruiseSpeed + correction, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
#else
    speedA = constrain(cruiseSpeed + correction, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    speedB = constrain(cruiseSpeed - correction, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
#endif
}


void calculateMotorSpeedFromJoystick(int xAxisValue, int yAxisValue, int* motor1, int* motor2) {
  // direction (X axis)
  // throttle  (Y axis)

  // handle throttle
  bool isForward = yAxisValue > 0;
  long x2 = (long)xAxisValue * (long)xAxisValue;
  long y2 = (long)yAxisValue * (long)yAxisValue;

  int throttle = constrain(sqrt( x2 + y2) , -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  throttle = map (throttle, -255, 255, 270, 270 + 180);
  throttle = sin( radians(throttle) ) * MOTOR_MAX_SPEED;

  float throttleRatio = (1.0 / MOTOR_MAX_SPEED) * throttle;



  int leftMotorSpeed  = 0;
  int rightMotorSpeed = 0;
  rightMotorSpeed = leftMotorSpeed  = throttleRatio * MOTOR_MAX_SPEED;


  // handle direction
  // only calc left & right if outside of X axis deadzone
  unsigned int xMag  = abs(xAxisValue);

  if ( xMag > XAXIS_DEADZONE  ) {
    bool isLeft = xAxisValue < 0;
    // adjust for the fact that the XAXIS_DEADZONE moves the start speed from 0 to XAXIS_DEADZONE
    int direction = map (xAxisValue+ (isForward?XAXIS_DEADZONE:-XAXIS_DEADZONE), -255-XAXIS_DEADZONE, 255+XAXIS_DEADZONE, -255,255);
    direction = map (xAxisValue, -255, 255, 270, 270 + 180);
    direction = sin( radians(direction) ) * MOTOR_MAX_SPEED;

    float directionRatio = abs((1.0 / MOTOR_MAX_SPEED) * direction);

    int speed = throttleRatio * MOTOR_MAX_SPEED;
    if (isLeft) {
      leftMotorSpeed  = (1.0 - directionRatio * 2) * speed;
      rightMotorSpeed = directionRatio * speed;
    } else {
      leftMotorSpeed  = directionRatio * speed;
      rightMotorSpeed = (1.0 - directionRatio * 2) * speed;
    }
  }

  // re-apply fwd/back sign.
  leftMotorSpeed = leftMotorSpeed * (isForward ? 1 : -1);
  rightMotorSpeed = rightMotorSpeed * (isForward ? 1 : -1);


  CB_DBG("%lu: xAxisValue,yAxisValue(%d,%d) =  throttle(%d),  Left,Right(%d,%d)", millis(), xAxisValue, yAxisValue, throttle, leftMotorSpeed, rightMotorSpeed);
#ifdef MOTOR_A_POS_IS_FORWARD
  *motor1 = rightMotorSpeed;
  *motor2 = leftMotorSpeed;
#else
  *motor2 = rightMotorSpeed;
  *motor1 = leftMotorSpeed;
#endif
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors and actuators


// read the IR sensors:
// set limit on reading. The reading can be very high and inaccurate on pitch black
void read_ir_sensors() {

  IRvals[0] = constrain(ANALOG_READ(IR1) - IRbias[0], 0, IR_MAX); //left looking from behind
  IRvals[1] = constrain(ANALOG_READ(IR2) - IRbias[1], 0, IR_MAX); //centre
  IRvals[2] = constrain(ANALOG_READ(IR3) - IRbias[2], 0, IR_MAX); //right
}

void motor(int _speedA, int _speedB) {
  // TODO: read config from eeprom
#ifdef BOT_TYPE_CUSTOM_PCB
motor_customPCB(_speedA, _speedB);
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


void printvalues ()
{
  static unsigned long lastPrint = millis();
  if ( millis() - lastPrint < 1000 )  return;
  lastPrint = millis();

  CB_DBG(    "%lu: IR(%u,%u,%u) Kpd(%d,%d) e(%d) PeDe(%d,%d) Sab(%d,%d) Mab(%d,%d), XY(%d,%d), MEM(%d), ", //VCC(%d)",
                 millis(),
                 IRvals[0], IRvals[1], IRvals[2],
                 Kp, Kd, error, P_error, D_error,
                 speedA, speedB, manualA, manualB,
                 xAxisValue, yAxisValue,
                 cb.getFreeMemory()
                 //ANALOG_READ(BATTERY_PIN)
            );
}

void lf_report_followingMode(bool isLineMode) {
  static unsigned long lastCall = millis();
  // throttle sending to 1000/x times a second
  if (millis() - lastCall > 500) {
    cb.callMethod(RACER_LINEFOLLOWING_MODE, isLineMode);
    lastCall = millis();
  }
}





//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Remote comms


void lf_updateMotorSpeeds(int _speedA, int _speedB, int _dummy) {
  //CB_DBG("%d,%d", _speedA, _speedB)
  manualA = _speedA;
}

void lf_updateAxis(int xAxis, int yAxis, int _dummy) {
  //CB_DBG("axis=%d,%d", xAxis, yAxis);
  xAxisValue = xAxis;          //joy X axis vale  = Direction  -255 to 255
  yAxisValue = yAxis;  //joy y axis vale = Throttle    -255 to 255
  if (isLineFollowingMode) {
    manualA = yAxisValue > 0 ? yAxisValue : 0;
  } else if ( (0 == xAxisValue) && (yAxisValue == 0) ) {
    manualA = manualB = 0;
    speedA = speedB = 0;
    motor(0, 0);                  // immediate stop
  } else {
    calculateMotorSpeedFromJoystick(xAxisValue, yAxisValue, &manualA, &manualB);
  }
}



void lf_updatePID(int _Kp, int _Ki, int _Kd) {
  CB_DBG("PID=%d,%d,%d", _Kp, _Ki, _Kd);
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
  cb.nvSetByte(NV_PID_P, Kp);
  cb.nvSetByte(NV_PID_I, Ki);
  cb.nvSetByte(NV_PID_D, Kd);
}

void lf_updateBias (int b1, int b2, int b3) {
  CB_DBG("Bias=%d,%d,%d", b1, b2, b3);
  IRbias[0] = b1;
  IRbias[1] = b2;
  IRbias[2] = b3;
  cb.nvSetByte(NV_IRBIAS_1, IRbias[0]);
  cb.nvSetByte(NV_IRBIAS_2, IRbias[1]);
  cb.nvSetByte(NV_IRBIAS_3, IRbias[2]);

}

void lf_updateLineFollowingMode(int _forceManualMode, int _d1, int _d2) {
  CB_DBG("ForceManual=%d", _forceManualMode)
  forceManualMode=_forceManualMode;
}

void lf_emitConfig(int _d1, int _d2, int _d3) {
  cb.callMethod(RACER_PID, Kp, Ki, Kd);
  cb.callMethod(RACER_IRBIAS, IRbias[0], IRbias[1], IRbias[2]);
}

void lf_emitIRValues(int v1, int v2, int v3) {
  static unsigned long lastCall = millis();
  if (millis() - lastCall > 200) {
    cb.callMethod(RACER_IRVALS, v1,v2,v3);
    lastCall = millis();
  }
}


void lf_ping(int v1) {
    //CB_DBG("ping", v1)
}



//////////////////////////////////////////////////////////////////////////////////////////////////

void cannybots_setup() {
  cb.setConfigStorage(NV_ID, NV_BASE);
  cb.registerHandler(RACER_CRUISESPEED, lf_updateMotorSpeeds);
  cb.registerHandler(RACER_LINEFOLLOWING_MODE, lf_updateLineFollowingMode);
  cb.registerHandler(RACER_PID, lf_updatePID);
  cb.registerHandler(RACER_IRBIAS, lf_updateBias);
  cb.registerHandler(RACER_JOYAXIS, lf_updateAxis);
  cb.registerHandler(RACER_CONFIG, lf_emitConfig);
  cb.registerHandler(RACER_IRVALS, lf_emitIRValues);
  cb.registerHandler(RACER_PING, lf_ping);
  cb.begin();
  getNVSettings();
}

void getNVSettings() {
  Kp=cb.nvGetByte(NV_PID_P);
  Ki=cb.nvGetByte(NV_PID_I);
  Kd=cb.nvGetByte(NV_PID_D);
  IRbias[0] = cb.nvGetByte(NV_IRBIAS_1);
  IRbias[1] = cb.nvGetByte(NV_IRBIAS_2);
  IRbias[2] = cb.nvGetByte(NV_IRBIAS_3);
 
}

void setNVDefaults() {
  cb.nvSetByte(NV_IRBIAS_1, 0 );
  cb.nvSetByte(NV_IRBIAS_2, 0 );
  cb.nvSetByte(NV_IRBIAS_3, 0 );
}

