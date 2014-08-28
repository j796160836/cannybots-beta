//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots LineFollowing Robot
//
// Authors:  Wayne Keenan & Anish Mampetta
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0  -  14.08.2014  -  Inital Version  (Wayne Keenan & Anish Mampetta)
// Version:   1.1  -  22.08.2014  -  RFduino only Version  (Wayne Keenan)
//
//////////////////////////////////////////////////////////////////////////////////////////////////
#include <RFduinoGZLL.h>

// Bot constants

////// Hardware Constants

// Infrared
#define IR_NUM_SENSORS               3

// TODO: change these to match wiring
#define IR1_PIN                      0
#define IR2_PIN                      1
#define IR3_PIN                      2

// Motor Pins
#define MOTOR_A1_PIN                 3
#define MOTOR_A2_PIN                 4
#define MOTOR_B1_PIN                 5
#define MOTOR_B2_PIN                 6
//#define MOTOR_MODE_PIN               7


////// Processing constants

#define IR1_BIAS                     0
#define IR2_BIAS                     0
#define IR3_BIAS                     0
#define IR_WHITE_THRESHOLD         800

#define MOTOR_MAX_SPEED            255
#define MOTOR_CRUISE_SPEED         150

#define OFF_LINE_MAX_TIME          200


#define PID_P                       65
#define PID_D                      145

#define PID_SCALE                100.0
#define PID_SAMPLE_TIME              7

#define JOYPAD_ID                    0
#define JOYPAD_AXIS_DEADZONE        20
#define JOYPAD_CONNECTION_TIMEOUT  200



//////////////////////////////////////////////////////////////////////////////////////////////////
/// Bot Variables

////// Inputs

// IR Sensors
int IRvals[IR_NUM_SENSORS] = {0};

// Joypad
int16_t  xAxisValue    = 0;              // (left) -255 .. 255 (right)
int16_t  yAxisValue    = 0;              // (down) -255 .. 255 (up)
bool     buttonPressed = 0;              // 0 = not pressed, 1 = pressed


////// Process / Algorithms

// PID
int Kp         = PID_P;
int Kd         = PID_D;
int P_error    = 0;
int D_error    = 0;
int error      = 0;
int error_last = 0;                                 // to calculate D_error = error - error_last
int correction = 0;


// LineFollowing State
bool isLineFollowingMode = false;
bool forceManualMode     = false;

// Timers in milli-seconds (1/1000 of a second)

unsigned long timeNow = millis();                    // the time at the start of the loop()
unsigned long pidLastTime = millis();                // when the PID was calculated last
unsigned long joypadLastTime = millis();             // the time the bot last received a joypad command
unsigned long offLineLastTime = millis();            // last time the bot came off the line
unsigned long offTheLineTime = 0;                    // how long has the bot been off the line, total since last leaving the line

////// Outputs

// speeds are  -255 (fullspeed back) ..  255 (full speed forward)

int cruiseSpeed = MOTOR_CRUISE_SPEED;      // default cruise speed when line following

// The current requested/calculated motor speeds
int speedA = 0;             // viewed from behind motor 'A' is on the left
int speedB = 0;             // viewed from behind motor 'B' is on the right



//////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino functions

void setup() {
  // Motor pins
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  motorSpeed(0, 0);
  RFduinoGZLL.hostBaseAddress = 0x91827364;

  RFduinoGZLL.begin(DEVICE0);
}


void loop() {
  //Serial.println(millis()-timeNow); // loop time delta
  timeNow = millis();
  readIRSensors();
  updateLineFollowingStatus();

  if (isLineFollowingMode) {
    calculatePID();
    joypadLineFollowingControlMode();
  } else {
    // in manual mode
    joypadManualControlMode();
  }
  motorSpeed(speedA, -speedB);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
// PID

void calculatePID() {
  // Calculate PID on a regular time basis
  if ((millis() - pidLastTime) < PID_SAMPLE_TIME ) {
    // return if called too soon
    return;
  }

  pidLastTime = millis();

  // process IR readings via PID
  error_last = error;                                   // store previous error before new one is caluclated
  error = IRvals[0] - IRvals[2];

  P_error = error * (Kp / PID_SCALE);                          // calculate proportional term
  D_error = (error - error_last) * (Kd / PID_SCALE);           // calculate differential term
  correction = P_error + D_error;
  speedA = cruiseSpeed - correction;
  speedB = cruiseSpeed + correction;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Joypad Handling

// This is called when the bot is on the line
// speedA and speedB will have already been set by pid_calculate() before this is run
void  joypadLineFollowingControlMode() {
  speedA = speedA + (yAxisValue / 2) - (xAxisValue / 64); //superpose yAxis with PID output speed
  speedB = speedB + (yAxisValue / 2) + (xAxisValue / 64);
}

// This is called when the bot is in manual mode.
// Use it to map the joypad X & Y axis (which both range from -255..255) to motor speeds (also -255..255)

void joypadManualControlMode() {
  // check if we have recently received joypad input
  if ( (timeNow - joypadLastTime) > JOYPAD_CONNECTION_TIMEOUT) {
    // no command has been received in the last X millis, err on the side of caution and stop!
    speedA = 0;
    speedB =  0;
    xAxisValue = 0;
    yAxisValue = 0;
  } else {
    // If the xis readings are small set them to 0
    if ( abs(xAxisValue) < JOYPAD_AXIS_DEADZONE)
      xAxisValue = 0;
    if ( abs(yAxisValue) < JOYPAD_AXIS_DEADZONE)
      yAxisValue = 0;

    speedA =  yAxisValue - xAxisValue / 2;
    speedB =  yAxisValue + xAxisValue / 2;
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Inputs

void readIRSensors() {
  IRvals[0] = analogRead(IR1_PIN) + IR1_BIAS; //left looking from behind
  IRvals[1] = analogRead(IR2_PIN) + IR2_BIAS; //centre
  IRvals[2] = analogRead(IR3_PIN) + IR3_BIAS; //right
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Outputs

void motorSpeed(int _speedA, int _speedB) {
  _speedA = constrain(_speedA, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  _speedB = constrain(_speedB, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  digitalWrite(MOTOR_A1_PIN, _speedA >= 0 ? HIGH : LOW) ;
  analogWrite (MOTOR_A2_PIN, abs(_speedA));
  digitalWrite(MOTOR_B1_PIN, _speedB >= 0 ? HIGH : LOW);
  analogWrite (MOTOR_B2_PIN, abs(_speedB));
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

void updateLineFollowingStatus() {
  if ((IRvals[1] <= IR_WHITE_THRESHOLD )) {
    offTheLineTime += timeNow - offLineLastTime;
    offLineLastTime = timeNow;

    if (offTheLineTime > OFF_LINE_MAX_TIME) {
      isLineFollowingMode = 0;
    }
  } else {
    offTheLineTime = 0;
    isLineFollowingMode = 1;
  }

  if (buttonPressed)
    forceManualMode = 1;
  else
    forceManualMode = 0;

  if (forceManualMode) {
    isLineFollowingMode = 0;
  }
}


void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  if (len >= 4) {
    int x = data[1];
    int y = data[2];
    buttonPressed = data[3];
    xAxisValue = map(x, 0, 255, -255, 255);
    yAxisValue = map(y, 0, 255, -255, 255);
    joypadLastTime = timeNow;                      // record the time we last received a joypad command.
  }
}

