//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Libraries
//#include <AnalogScanner.h>
#include <EEPROM.h>


#include <SPI.h>
#include <Cannybots.h>
#include "CannybotsRacer.h"
Cannybots& cb = Cannybots::getInstance();




//////////////////////////////////////////////////////////////////////////////////////////////////
//#define USE_ANALOG_LIB
//#define USE_IR_WAYPOINT_DETECTION



//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_ANALOG_LIB
// see: https://github.com/merose/AnalogScanner
#include <AnalogScanner.h>
AnalogScanner scanner;
#define ANALOG_READ scanner.getValue
#else
#define ANALOG_READ analogRead
#define ANALOG_READING_DELAY 10
#endif





//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Config

// PID
#define PID_METHOD_1

#define PID_SETPOINT        0.0
#define PID_SAMPLE_TIME     10
#define PID_DIV             10

#define PRINTVALS_INTERVAL   1000

#define OFF_LINE_MAX_TIME 200    

#define MANUAL_MODE_RADIOSILENCE_TIMEOUT 500

// Motor settings
#define NUM_MOTORS       2
#define MOTOR_MAX_SPEED 255
#define MOTOR_TEST_SPEED MOTOR_MAX_SPEED/2
//#define MOTOR_A_IS_ON_RIGHT

#define MOTOR_A_POS_IS_FORWARD 1
#define MOTOR_B_POS_IS_FORWARD 0

// number of divisions between current and target motor speed
#define MAX_MOTOR_DELTA_DIVISOR     100.0
// max motor speed change
#define MAX_MOTOR_DELTA             255.0


// Joystick
#define XAXIS_DEADZONE 50


// IR Sensor settings
#define NUM_IR_SENSORS   3

// Battery Voltage sensing
#ifdef BOT_TYPE_CUSTOM_PCB
#define BATTERY_PIN A1
#endif



// Pinout wiring

// Aiva
#define BOT_TYPE_CUSTOM_PCB 1
#define IR_MAX 1000
#define WHITE_THRESHOLD 700
#define IR1 A6
#define IR2 A8
#define IR3 A11
#define IR_WAYPOINT_DETECTION_PIN IR3

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


// Indicator LED
#define STATUS_LED 13

// waypoint detection

// Ignore this for now
#ifdef USE_IR_WAYPOINT_DETECTION
//#include <IRremoteORG.h>
//IRrecv irrecv(IR_WAYPOINT_DETECTION_PIN);
#endif



//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// App State

bool isLineFollowingMode = true;
bool forceManualMode = false;
int baseCruiseSpeed = 150;
int cruiseSpeed = baseCruiseSpeed;

int speedA = 0;
int speedB = 0;

// speed only used in in manual mode:
int manualA = 0;  
int manualB = 0;
//int speeds[NUM_MOTORS];

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


// ignore this for now.
#ifdef USE_ANALOG_LIB
  //int scanOrder[NUM_IR_SENSORS + 1] = {IR1, IR2, IR3, BATTERY_PIN};
  int scanOrder[NUM_IR_SENSORS] = {IR1, IR2, IR3};
  scanner.setScanOrder(NUM_IR_SENSORS, scanOrder);
  scanner.beginScanning();  
#endif

// ignore this for now.
#ifdef USE_IR_WAYPOINT_DETECTION
  irrecv.enableIRIn();
#endif

  setup_PID();
  mycannybots_setup();
}


void loop() {
  // do some stats...  
  loopcount++;
  loopNowTime = millis();
  loopDeltaTime = loopNowTime - loopLastTime;
  loopLastTime = loopNowTime;
  
  // read and publish R sensor values
  read_ir_sensors();
  lf_emitIRValues(IRvals[0], IRvals[1], IRvals[2]);

  
  // count up the time spent off the line, rahter than switching to manual mode the instance the mid sensor goes of the line
  if ((IRvals[1] <= WHITE_THRESHOLD )) {

    offTheLineTime += loopNowTime - offLineLastTime;
    offLineLastTime = loopNowTime;

    if (offTheLineTime > OFF_LINE_MAX_TIME) {
      if (resetSpeed) {
        speedA = speedB = manualA = manualB = 0;
        resetSpeed = false;
      }
      isLineFollowingMode = 0;
    }

  } else {
    offTheLineTime = 0;
    isLineFollowingMode = 1;
    resetSpeed = true;
  }
  

  if (forceManualMode) {
    isLineFollowingMode = 0;
  }

  lf_report_followingMode(isLineFollowingMode);

  if (isLineFollowingMode) {
    enable_PID();
    calculate_PID();
  } else {
    // in manual mode
    disable_PID();
    if ( (millis() - cb.getLastInboundCommandTime()) > MANUAL_MODE_RADIOSILENCE_TIMEOUT) {
      // no command has been received in the last X millis, err on the side of caution and stop!
      speedA = speedB =  0;
    } else {
      // rate limit/ease the speed change
      speedA = speedA + min((manualA - speedA) / MAX_MOTOR_DELTA_DIVISOR, MAX_MOTOR_DELTA * sign(manualA - speedA));
      speedB = speedB + min((manualB - speedB) / MAX_MOTOR_DELTA_DIVISOR, MAX_MOTOR_DELTA * sign(manualB - speedB));
    }
  }
  motor(speedA, speedB);
  printvalues();

  cb.update();
//ignore this for now
#ifdef USE_IR_WAYPOINT_DETECTION
  irwaypoint_loop();
#endif
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////
// Joystick to motor speed conversion

void calculateMotorSpeedFromJoystick(int xAxisValue, int yAxisValue, int* motor1, int* motor2) {
  // direction (X axis) -255 .. +255
  // throttle  (Y axis) -255 .. +255

  // handle throttle
  bool isForward = yAxisValue > 0;
  long x2 = (long)xAxisValue * (long)xAxisValue;
  long y2 = (long)yAxisValue * (long)yAxisValue;

  // get the vecor length, constrain it to max motor speed and then remap to a phase angle range more suitable to an accelration curve
  int throttle = constrain(sqrt( x2 + y2) , -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  throttle = map (throttle, -255, 255, 270, 270 + 180);
  throttle = sin( radians(throttle) ) * MOTOR_MAX_SPEED;

  float throttleRatio = (1.0 / MOTOR_MAX_SPEED) * throttle;

  int leftMotorSpeed  = 0;
  int rightMotorSpeed = 0;
  rightMotorSpeed = leftMotorSpeed  = throttleRatio * MOTOR_MAX_SPEED;


  // handle direction
  // only calc left & right if outside of X axis deadzone (touch screens are terrible feedback devices, so be generous!)
  unsigned int xMag  = abs(xAxisValue);

  if ( xMag > XAXIS_DEADZONE  ) {
    bool isLeft = xAxisValue < 0;
    // adjust for the fact that the XAXIS_DEADZONE moves the start speed from 0 to XAXIS_DEADZONE
    int direction = map (xAxisValue + (isForward ? XAXIS_DEADZONE : -XAXIS_DEADZONE), -255 - XAXIS_DEADZONE, 255 + XAXIS_DEADZONE, -255, 255);
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
  } // nothing, was outside X axis stick deadzone

  // re-apply fwd/back sign from
  leftMotorSpeed = leftMotorSpeed * (isForward ? 1 : -1);
  rightMotorSpeed = rightMotorSpeed * (isForward ? 1 : -1);


  CB_DBG("%lu: xAxisValue,yAxisValue(%d,%d) =  throttle(%d),  Left,Right(%d,%d)", millis(), xAxisValue, yAxisValue, throttle, leftMotorSpeed, rightMotorSpeed);
#ifdef MOTOR_A_IS_ON_RIGHT
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

#ifndef USE_ANALOG_LIB
  //ANALOG_READ(IR1); delay(ANALOG_READING_DELAY);
#endif  
  IRvals[0] = constrain(ANALOG_READ(IR1) - IRbias[0], 0, IR_MAX); //left looking from behind

#ifndef USE_ANALOG_LIB
  //ANALOG_READ(IR2); delay(ANALOG_READING_DELAY);
#endif  
  IRvals[1] = constrain(ANALOG_READ(IR2) - IRbias[1], 0, IR_MAX); //centre
  
#ifndef USE_ANALOG_LIB
  //ANALOG_READ(IR3); delay(ANALOG_READING_DELAY);
#endif  
  IRvals[2] = constrain(ANALOG_READ(IR3) - IRbias[2], 0, IR_MAX); //right

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
    cb.callMethod(RACER_LINEFOLLOWING_MODE, isLineMode);
    lastCall = millis();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Stored Settings  (EEPROM/Flash)

void getPIDSettings() {
  setPID_P(cb.nvGetInt(NV_PID_P));
  setPID_I(cb.nvGetInt(NV_PID_I));
  setPID_D(cb.nvGetInt(NV_PID_D));
  IRbias[0] = cb.nvGetByte(NV_IRBIAS_1);
  IRbias[1] = cb.nvGetByte(NV_IRBIAS_2);
  IRbias[2] = cb.nvGetByte(NV_IRBIAS_3);
}

void setNVDefaults() {
  cb.nvSetInt(NV_PID_P, 0);
  cb.nvSetInt(NV_PID_I, 0);
  cb.nvSetInt(NV_PID_D, 0);
  cb.nvSetByte(NV_IRBIAS_1, 0 );
  cb.nvSetByte(NV_IRBIAS_2, 0 );
  cb.nvSetByte(NV_IRBIAS_3, 0 );
}




//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Remotely called funcs


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
  update_PID(_Kp, _Ki, _Kd);

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
  CB_DBG("ForceManual=%d", _forceManualMode);
  forceManualMode = _forceManualMode;
}

void lf_emitConfig(int _d1, int _d2, int _d3) {
  cb.callMethod(RACER_PID, getPID_P(), getPID_I(), getPID_D());
  cb.callMethod(RACER_IRBIAS, IRbias[0], IRbias[1], IRbias[2]);
}

void lf_emitIRValues(int v1, int v2, int v3) {
  static unsigned long lastCall = millis();
  if (millis() - lastCall > 200) {
    cb.callMethod(RACER_IRVALS, v1, v2, v3);
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
// IR Waypoint detection

#ifdef USE_IR_WAYPOINT_DETECTION

// see:
decode_results results;

// Dumps out the decode_results structure.
// Call this after IRrecv::decode()
// void * to work around compiler issue
void irwaypoint_dump(void *v) {
  decode_results *results = (decode_results *)v;
  //void irwaypoint_dump(decode_results *results) {
  int count = results->rawlen;
  Serial.print(results->value, HEX);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
  Serial.print("Raw (");
  Serial.print(count, DEC);
  Serial.print("): ");

  for (int i = 0; i < count; i++) {
    if ((i % 2) == 1) {
      Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
    }
    else {
      Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
    }
    Serial.print(" ");
  }
  Serial.println("");
}


void irwaypoint_loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irwaypoint_dump(&results);
    irrecv.resume(); // Receive the next value
  }
}
#endif // USE_IR_WAYPOINT_DETECTION


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots glulogic

void mycannybots_setup() {
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
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
/// Testing

void test(int16_t p1, int16_t p2, int16_t p3) {

  switch (p1) {
    case CANNYBOTSRACER_TEST_MOTORS:
      motor(MOTOR_TEST_SPEED, 0);
      delay(500);
      motor(-MOTOR_TEST_SPEED, 0);
      delay(500);
      motor(0, MOTOR_TEST_SPEED);
      delay(500);
      motor(0, -MOTOR_TEST_SPEED);
      delay(500);
      motor(0, 0);
      break;
    default:
      break;
  }
}


