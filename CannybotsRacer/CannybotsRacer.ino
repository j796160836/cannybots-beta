//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Libraries

#include <Cannybots.h>
#include "CannybotsRacer.h"
Cannybots& cb = Cannybots::getInstance();


//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef USE_ANALOG_LIB
// see: https://github.com/merose/AnalogScanner
#include <AnalogScanner.h>
AnalogScanner scanner;
#define ANALOG_READ scanner.getValue
#else
#define ANALOG_READ analogRead
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Config

// Pinout wiring

#define IR1 A8
#define IR2 A9
#define IR3 A10

#define pinA1 6 
#define pinA2 5
#define pinB1 4
#define pinB2 3
#define pin_MODE 7

// IR Sensor settings

#define NUM_IR_SENSORS   3
#define IR_MAX 1000
#define WHITE_THRESHOLD 700

// Motor settings
#define NUM_MOTORS       2
#define MOTOR_MAX_SPEED 255

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// App State

// Pid
int Kp, Ki, Kd;
int I_limit = 100;
int P_error = 0;
int I_error = 0;
int D_error = 0;
int error = 0;
int error_last = 0; // to calculate D_error = error - error_last
int correction = 0; //error after PID filter

bool isLineFollowingMode = true;

int baseCruiseSpeed = 80;
int cruiseSpeed = baseCruiseSpeed;

int speedA = 0;
int speedB = 0;
int manualA = 0;
int manualB = 0;
//int speeds[NUM_MOTORS];

int yAxisValue = 0;
int xAxisValue = 0;

int IRval[NUM_IR_SENSORS];
int IRbias[NUM_IR_SENSORS];

volatile unsigned long lastCommandTime = millis();

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
  pinMode(pin_MODE, OUTPUT);
  digitalWrite(pin_MODE, HIGH); //to set controller to Phase/Enable mode
  digitalWrite(13, HIGH);
  cannybots_setup();
}


void loop() {
  read_ir_sensors();

  isLineFollowingMode =  IRval[1] >= WHITE_THRESHOLD;
  lf_report_followingMode(isLineFollowingMode);

  if (isLineFollowingMode) {
    calculate_PID();
  } else {
    // in manual mode
    if ((millis() - lastCommandTime) > 2000) {
      // no command has been received in the last 2 seconds, err on the side of caution and stop!
      speedA = speedB = 0;
    } else {
      speedA = manualA;
      speedB = manualB;
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


void calculate_PID() {
  // process IR readings via PID
    error_last = error;                                   // store previous error before new one is caluclated
    error = constrain(IRval[0] - IRval[2], -30, 30);        // set bounds for error
    P_error = error * Kp / 10;                               // calculate proportional term
    D_error = (error - error_last) * Kd / 10;                // calculate differential term
    correction = P_error + D_error;
    cruiseSpeed = baseCruiseSpeed + manualA;
    speedA = constrain(cruiseSpeed - correction, -200, 200);
    speedB = constrain(cruiseSpeed + correction, -200, 200);
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

  if ( xMag > 25  ) {
    bool isLeft = xAxisValue < 0;
    int direction = map (xAxisValue, -255, 255, 270, 270 + 180);
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


  //CB_DBG("%lu: xAxisValue,yAxisValue(%d,%d) =  throttle(%d),  Left,Right(%d,%d) - freemem=(%d)", millis(), xAxisValue, yAxisValue, throttle, leftMotorSpeed, rightMotorSpeed, cb.getFreeMemory());

  *motor1 = rightMotorSpeed;
  *motor2 = leftMotorSpeed;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors and actuators


// read the IR sensors:
// set limit on reading. The reading can be very high and inaccurate on pitch black
void read_ir_sensors() {

  IRval[0] = constrain(ANALOG_READ(IR1) - IRbias[0], 0, IR_MAX); //left looking from behind
  IRval[1] = constrain(ANALOG_READ(IR2) - IRbias[1], 0, IR_MAX); //centre
  IRval[2] = constrain(ANALOG_READ(IR3) - IRbias[2], 0, IR_MAX); //right
}


void motor(int _speedA, int _speedB)
{
  _speedA = constrain(_speedA, -255, 255);
  _speedB = constrain(_speedB, -255, 255);

  digitalWrite(pinA1, _speedA >= 0 ? HIGH : LOW) ;
  analogWrite (pinA2, abs(_speedA));

  digitalWrite(pinB1, _speedB >= 0 ? LOW : HIGH);
  analogWrite (pinB2, abs(_speedB));
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
// Utils


void printvalues ()
{
  static unsigned long lastPrint = millis();
  if ( millis() - lastPrint < 250 )  return;
  lastPrint = millis();

  CB_DBG(    "%lu: IR(%u,%u,%u) Kpd(%d,%d) e(%d) PeDe(%d,%d) Sab(%d,%d) Mab(%d,%d), XY(%d,%d), MEM(%d)",
                 millis(),
                 IRval[0], IRval[1], IRval[2],
                 Kp, Kd, error, P_error, D_error,
                 speedA, speedB, manualA, manualB,
                 xAxisValue, yAxisValue,
                 cb.getFreeMemory()
            );
}





//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Remote comms


void cannybots_setup() {
  cb.setConfigStorage("CBLF", 64);
  cb.registerHandler(RACER_CRUISESPEED, lf_updateMotorSpeeds);
  cb.registerHandler(RACER_LINEFOLLOWING_MODE, lf_updateLineFollowingMode);
  cb.registerHandler(RACER_PID, lf_updatePID);
  cb.registerHandler(RACER_IRBIAS, lf_updateBias);
  cb.registerHandler(RACER_JOYAXIS, lf_updateAxis);
  cb.registerHandler(RACER_CONFIG, lf_emitConfig);
  cb.begin();
}

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
}

void lf_updateBias (int b1, int b2, int b3) {
  CB_DBG("Bias=%d,%d,%d", b1, b2, b3);
  IRbias[0] = b1;
  IRbias[1] = b2;
  IRbias[2] = b3;
}

void lf_updateLineFollowingMode(int isLFMode, int _d1, int _d2) {
  CB_DBG("LF=%d", isLFMode)
  isLineFollowingMode=isLFMode;
}

void lf_emitConfig(int _d1, int _d2, int _d3) {
  cb.callMethod(RACER_PID, 1, 2, 3);
  cb.callMethod(RACER_IRBIAS, 4, 5, 6);
}




