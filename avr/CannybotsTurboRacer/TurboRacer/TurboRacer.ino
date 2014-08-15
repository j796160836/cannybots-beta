//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots LineFollowing Robot
//
// Authors:  Wayne Keenan & Anish Mampetta
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0  -  14.08.2014  -  Inital Version  (Wayne Keenan & Anish Mampetta)
//
//////////////////////////////////////////////////////////////////////////////////////////////////

// Bot constants

////// Hardware Constants

// Infrared
#define IR_NUM_SENSORS               3
#define IR1_PIN                     A6
#define IR2_PIN                     A8
#define IR3_PIN                    A11

// Motor Pins
#define MOTOR_A1_PIN                 3
#define MOTOR_A2_PIN                 5
#define MOTOR_B1_PIN                 6
#define MOTOR_B2_PIN                 9 
#define MOTOR_MODE_PIN               2

// Flashy Lights
#define STATUS_LED_PIN              13

////// Processing constants

#define IR1_BIAS                     0
#define IR2_BIAS                     0
#define IR3_BIAS                     0
#define IR_WHITE_THRESHOLD         700

#define MOTOR_MAX_SPEED            255
#define MOTOR_CRUISE_SPEED         120

#define OFF_LINE_MAX_TIME            0

#define PID_P                        5
#define PID_D                        1
#define PID_SAMPLE_TIME              0
#define PID_SCALE                 10.0

#define JOYPAD_ID                    0
#define JOYPAD_AXIS_DEADZONE        20
#define JOYPAD_CONNECTION_TIMEOUT  200

#define JOYPAD_LINEFOLLOW_MODE_SCALE 3
#define JOYPAD_MANUAL_MODE_SCALE     4


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
  // Serial setup
  Serial.begin(9600);            // USB serial port (debugging)
  Serial1.begin(9600);           // Data from the RFDuino is read from here (RFduino can't go faster than 9600 in BLE mode)

  // Motor pins
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(MOTOR_MODE_PIN, OUTPUT);
  digitalWrite(MOTOR_MODE_PIN, HIGH);   //to set controller to Phase/Enable mode
  motorSpeed(0,0);

  // Headlights...
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
}


void loop() {
  timeNow = millis();

  readSerial();
  readIRSensors();
  updateLineFollowingStatus();

  if (isLineFollowingMode) {
    calculatePID();
    joypadLineFollowingControlMode();
  } else {    
     // in manual mode
    joypadManualControlMode();
  }
  motorSpeed(speedA, speedB);
  printVals();
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
  error_last = error;                                   // store previous error before new one is caluclated
  error = IRvals[0] - IRvals[2];                        

  P_error = error * Kp / PID_SCALE;                          // calculate proportional term
  D_error = (error - error_last) * Kd / PID_SCALE;           // calculate differential term
  correction = P_error + D_error;
  speedA = cruiseSpeed + correction;
  speedB = cruiseSpeed - correction;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Joypad Handling

// This is called when the bot is on the line
// speedA and speedB will have already been set by pid_calculate() before this is run
void  joypadLineFollowingControlMode() {
  speedA = speedA + (yAxisValue / JOYPAD_LINEFOLLOW_MODE_SCALE); //superpose yAxis with PID output speed
  speedB = speedB + (yAxisValue / JOYPAD_LINEFOLLOW_MODE_SCALE);
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

    speedA =  (yAxisValue + xAxisValue) / JOYPAD_MANUAL_MODE_SCALE;
    speedB =  (yAxisValue - xAxisValue) / JOYPAD_MANUAL_MODE_SCALE;
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

void printVals() {
  static unsigned long lastPrint = millis();
  if (millis() - lastPrint < 100) {
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
  Serial.print("),Joy(X,Y,Button)=(");
  Serial.print(xAxisValue, DEC);
  Serial.print(",");
  Serial.print(yAxisValue, DEC);
  Serial.print(",");
  Serial.print(buttonPressed, DEC);
  Serial.print("),Speed(A,B)=(");
  Serial.print(speedA, DEC);
  Serial.print(",");
  Serial.print(speedB, DEC);
  Serial.println(")");
}

// Serial Input
// We're expecting messages of 6 bytes in the form:  >>IXYB
// Where;
// >> = start marker, as-is
// I = joypad ID  (0-7 for GZLL joypad ID's)
// X = unsigned byte for xAxis:          0 .. 255 mapped to -254 .. 254
// Y = unsigned byte for yAxis:          0 .. 255 mapped to -254 .. 254
// B = unsigned byte for button pressed: 0 = no, 1 = yes
void readSerial() {
  static int c = 0, lastChar = 0;
  while (Serial1.available() >= 6) {
    lastChar = c;
    c = Serial1.read();
    if ( ('>' == c) && ('>' == lastChar) ) {
      if (JOYPAD_ID == Serial1.read()) {
        xAxisValue    = Serial1.read();
        yAxisValue    = Serial1.read();
        buttonPressed = Serial1.read();
        xAxisValue = map(xAxisValue, 0, 255, -255, 255);
        yAxisValue = map(yAxisValue, 0, 255, -255, 255);
      } else {
        // ignore the data
        Serial1.read();
        Serial1.read();
        Serial1.read();
      }
      lastChar = c = 0;
      joypadLastTime = timeNow;                      // record the time we last received a joypad command.
    }
  }
}

