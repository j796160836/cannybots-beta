//////////////////////////////////////////////////////////////////////////////////////////////////
//
// CannyBots LineFollowing Robot
//
// Authors:  Wayne Keenan & Anish Mampetta
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0  -  14.08.2014  -  Inital Version  (wayne@cannybots.com, mampetta@cannybots.com)
// Version:   1.1  -  15.08.2014  -  Tidied naming                  (wayne@cannybots.com)
// Version:   1.2  -  16.08.2014  -  Added sending serial messages  (wayne@cannybots.com)
// Version:   1.3  -  16.08.2014  -  Added sending/receving key/value pairs  (wayne@cannybots.com)
// Version:   1.4  -  17.08.2014  -  Use binary encoding for message variables
//
//////////////////////////////////////////////////////////////////////////////////////////////////

// Bot constants

#define BOT_ID                     128

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

#define JOYPAD_ID                    1
// Anish Joypad     =  0
// Waynes barebones =  1
// Default iOS app  = 10
// Wayne iPhone App = 11
#define JOYPAD_AXIS_DEADZONE        20
#define JOYPAD_CONNECTION_TIMEOUT  200


// Interval timer constants (milliseconds)
#define PRINT_DEBUG_INTERVAL      1000

#define IR_SEND_INTERVAL           50

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

// Lap Timing
unsigned long currentStartLapTime = 0;
int  lapCount = 0;

// Requested actions state

bool sendIR     = false;

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
  motorSpeed(0, 0);

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
  motorSpeed(speedA * 1, speedB * 1);
  sendSensorReadings();

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
  speedA = speedA + (yAxisValue / 3); //superpose yAxis with PID output speed
  speedB = speedB + (yAxisValue / 3);
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
    if ( abs(xAxisValue) < JOYPAD_AXIS_DEADZONE) {
      xAxisValue = 0;
    }
    if ( abs(yAxisValue) < JOYPAD_AXIS_DEADZONE) {
      yAxisValue = 0;
    }

      speedA =  (yAxisValue + xAxisValue) / 4;
      speedB =  (yAxisValue - xAxisValue) / 4;
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
  if (millis() - lastPrint < PRINT_DEBUG_INTERVAL) {
    return;
  }
  lastPrint = timeNow;

  if (!Serial)
    return;
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
  Serial.print("),PID(P,D)=(");
  Serial.print(Kp, DEC);
  Serial.print(",");
  Serial.print(Kd, DEC);
  Serial.print("),Speed(A,B)=(");
  Serial.print(speedA, DEC);
  Serial.print(",");
  Serial.print(speedB, DEC);
  Serial.println(")");
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Lap Timing

// this should be called at least once to inform the client (e.g. phone) that the bot has started racing
void lap_started() {
  currentStartLapTime = timeNow;
  writeData("LSTRT",  currentStartLapTime);
  writeData("LAPS", lapCount);
  lapCount++;
}

// call on lap complete, updates phone with current lap time & lap count, it then restarts the lap timer and incremetn the laptcount
void lap_completed() {
  writeData("LTIME",  timeNow - currentStartLapTime);
  writeData("LAPS", lapCount);
  currentStartLapTime = timeNow;
}

// this should be called to tell the client (e.g. phone app) that lap timing and counting has finished
void lap_stopTiming() {
  writeData("LEND",  true);
}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Message Handling

//// Sending variable update out

void sendSensorReadings() {
  if (!sendIR) {
    return;
  }
  static unsigned long irSendLastTime = millis();
  if ( (timeNow - irSendLastTime) < IR_SEND_INTERVAL) {
    return;
  }
  irSendLastTime = timeNow;
  writeData("IRVAL", IRvals[0], IRvals[1], IRvals[2]);
}

// Receiving variable updates

// Helper macro(s)
#define variableNameMatches(a,b) (strncmp(a,b,5)==0)

// read in the serial data, return number of bytes consumed.
void updateVariable(const char* name, int* count) {

  if (variableNameMatches(name, "JOY01")) {
    xAxisValue    = readInt(count);
    yAxisValue    = readInt(count);
    buttonPressed = readInt(count);
  } else if (variableNameMatches(name, "PID_P")) {
    Kp = readInt(count);
  } else if (variableNameMatches(name, "PID_D")) {
    Kd = readInt(count);
  } else if (variableNameMatches(name, "GETCF")) {
    writeData("PID", Kp, Kd);
  } else if (variableNameMatches(name, "SNDIR")) {
    sendIR = readInt(count);
  } else {
    // unrecognised variable name, ignore.
  }
}


