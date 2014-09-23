//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots LineFollowing Robot - RFduino
//
// Authors:  Wayne Keenan & Anish Mampetta
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0  -  22.09.2014  -  Inital Version  (Wayne Keenan & Anish Mampetta)
//
//////////////////////////////////////////////////////////////////////////////////////////////////

// uncomment this to get debug message on serial 
// #define DEBUG
// Note that it will interfer with the operation of the motor driver, still useful though.

#include <RFduinoGZLL.h>
#include <RFduinoBLE.h>

// Bot constants

////// Hardware Constants

// Infrared
#define IR_NUM_SENSORS               3

#define IR1_PIN                      6
#define IR2_PIN                      5
#define IR3_PIN                      4

// Motor Pins
#define MOTOR_A1_PIN                 0
#define MOTOR_A2_PIN                 3
#define MOTOR_B1_PIN                 1
#define MOTOR_B2_PIN                 2


////// Processing constants

#define IR1_BIAS                     0
#define IR2_BIAS                     0
#define IR3_BIAS                     0
#define IR_WHITE_THRESHOLD         750

#define MOTOR_MAX_SPEED            255
#define MOTOR_CRUISE_SPEED         120

#define OFF_LINE_MAX_TIME            0

#define PID_P 25
#define PID_D 250
#define PID_SAMPLE_TIME              5


#define JOYPAD_ID 0
#define JOYPAD_AXIS_DEADZONE 10

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Bot Variables

////// Inputs

// IR Sensors
int IRvals[IR_NUM_SENSORS] = {0};

// Joypad
volatile int16_t  xAxisValue    = 0;              // (left) -255 .. 255 (right)
volatile int16_t  yAxisValue    = 0;              // (down) -255 .. 255 (up)
volatile bool     buttonPressed = 0;              // 0 = not pressed, 1 = pressed


////// Process / Algorithms

// PID
int Kp = PID_P;
int Ki = 0;
int Kd = PID_D;
int P_error = 0;
int I_sum = 0;
int I_error = 0;
int D_error = 0;
int error = 0;
int error_last = 0;                                 // to calculate D_error = error - error_last
int correction = 0;


// LineFollowing State
bool isLineFollowingMode = false;
bool forceManualMode     = false;

// Timers in milli-seconds (1/1000 of a second)

unsigned long timeNow = millis();                    // the time at the start of the loop()
unsigned long pidLastTime = millis();                // when the PID was calculated last
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
#ifdef DEBUG
  Serial.begin(9600);
#endif

  //  dumpAnalogReadingsForAllPins();

  // Motor pins
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  //motorTest();

  radio_setup();
}

void loop() {
  radio_loop();

  timeNow = millis();

  readIRSensors();
  updateLineFollowingStatus();

  if (isLineFollowingMode) {
    calculatePID();
    joypadLineFollowingControlMode();
  } else {
    // in manual mode
    joypadManualControlMode();
    I_sum = 0; //set integral sum to zero
  }
  motorSpeed(speedA, speedB);
#ifdef DEBUG
  printVals();
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// PID

void calculatePID() {
  // Calculate PID on a regular time basis
  if ((timeNow - pidLastTime) < PID_SAMPLE_TIME ) {
    // return if called too soon
    return;
  }
  pidLastTime = timeNow;
  // process IR readings via PID
  Kp = PID_P;
  Ki = 0;
  Kd = PID_D;
  error_last = error; // store previous error value before new one is caluclated
  error = IRvals[0] - IRvals[2];
  P_error = error * Kp / 100.0; // calculate proportional term
  I_sum = constrain (I_sum + error, -1000, 1000); // integral term
  I_error = I_sum * Ki / 100.0;
  D_error = (error - error_last) * Kd / 100.0;          // calculate differential term
  correction = P_error + D_error + I_error;
  speedA = cruiseSpeed - correction;
  speedB = cruiseSpeed + correction;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Joypad Handling

// This is called when the bot is on the line
// speedA and speedB will have already been set by pid_calculate() before this is run
void  joypadLineFollowingControlMode() {
  speedA = speedA + ((yAxisValue - (xAxisValue / 2)) / 2.0); //superpose yAxis with PID output speed
  speedB = speedB + ((yAxisValue + (xAxisValue / 2)) / 2.0);
}

// This is called when the bot is in manual mode.
// Use it to map the joypad X & Y axis (which both range from -255..255) to motor speeds (also -255..255)

void joypadManualControlMode() {

    // If the xis readings are small set them to 0
    if ( abs(xAxisValue) < JOYPAD_AXIS_DEADZONE)
      xAxisValue = 0;
    if ( abs(yAxisValue) < JOYPAD_AXIS_DEADZONE)
      yAxisValue = 0;

    //speedA =  (yAxisValue + (xAxisValue / 2)) / 3.0;
    //speedB =  (yAxisValue - (xAxisValue / 2)) / 3.0;
    speedA =  xAxisValue;
    speedB =  -yAxisValue;

}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Inputs

// Sensors
void readIRSensors() {
  IRvals[0] = analogRead(IR1_PIN) + IR1_BIAS; //left looking from behind
  IRvals[1] = analogRead(IR2_PIN) + IR2_BIAS; //centre
  IRvals[2] = analogRead(IR3_PIN) + IR3_BIAS; //right
}

// Joypad

// this is called when a data packet is received via the radio, do very little processing, just set some vars.
// NOTE: because this is called as a background event then any printed value may be more recent than the value used in the prior PID calc.
void joypad_update(int x, int y, int b) {
  xAxisValue = x;
  yAxisValue = y;
  buttonPressed = b;
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

void printVals() {
  static unsigned long lastPrint = millis();
  if (millis() - lastPrint < 200) {
    return;
  }
  lastPrint = millis();

  Serial.print(timeNow);
  Serial.print(":IR=(");
  Serial.print(IRvals[0], DEC);
  Serial.print(",");
  Serial.print(IRvals[1], DEC);
  Serial.print(",");
  Serial.print(IRvals[2], DEC);
  Serial.print("),errors =(");
  Serial.print(P_error, DEC);
  Serial.print(",");
  Serial.print(D_error, DEC);
  Serial.print(",");
  Serial.print(I_error, DEC);
  Serial.print(",");
  Serial.print("),Speed(A,B)=(");
  Serial.print(speedA, DEC);
  Serial.print(",");
  Serial.print(speedB, DEC);
  Serial.print("),Joy(X,Y,Button)=(");
  Serial.print(xAxisValue, DEC);
  Serial.print(",");
  Serial.print(yAxisValue, DEC);
  Serial.print(",");
  Serial.print(buttonPressed, DEC);
  Serial.println(")");
}

#ifdef DEBUG
void dumpAnalogReadingsForAllPins() {
  while (1) {
    for (int pin = 2; pin < 7; pin++) {
      analogRead(pin);
      Serial.print(pin, DEC);
      Serial.print("=");
      Serial.print( analogRead(pin), DEC);
      Serial.print("\t");
    }
    Serial.println("\t");
  }
}

#endif
void motorTest() {
  motorSpeed(128, 0);
  delay(1000);
  motorSpeed(-128, 0);
  delay(1000);
  motorSpeed(0, 128);
  delay(1000);
  motorSpeed(0, -128);
  delay(1000);
  motorSpeed(0, 0);
}

