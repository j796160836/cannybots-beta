//////////////////////////////////////////////////////////////////////////////////////////////////
// Bot constants

////// Hardware Constants

// Infrared
#define NUM_IR_SENSORS 3
#define IR1 A6
#define IR2 A8
#define IR3 A11
#define IR1_BIAS 0
#define IR2_BIAS 0
#define IR3_BIAS 0

// Motor Pins
#define pinA1 3
#define pinA2 5
#define pinB1 6
#define pinB2 9
#define pinMODE 2

// Flashy Lights
#define STATUS_LED 13


////// Processing constants

#define WHITE_THRESHOLD                    700
#define OFF_LINE_MAX_TIME                    0
#define PID_SAMPLE_TIME                      0
#define JOYPAD_AXIS_DEADZONE                20
#define MANUAL_MODE_RADIOSILENCE_TIMEOUT   200

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Bot Variables

////// Inputs

// IR Sensors
int IRvals[NUM_IR_SENSORS] = {0};

// Joypad
int16_t  xAxisValue    = 0;              // (left) -255 .. 255 (right)
int16_t  yAxisValue    = 0;              // (down) -255 .. 255 (up)
bool     buttonPressed = 0;              // 0 = not pressed, 1 = pressed


////// Process / Algorithms

int Kp = 3;
int Ki = 0;
int Kd = 1;
int P_error = 0, D_error = 0;
int error = 0;
int error_last = 0;                                     // to calculate D_error = error - error_last
int correction = 0;


// LineFollowing State
bool isLineFollowingMode = false;
bool forceManualMode     = false;

// Timers (in milli seconds 1/1000 of a second)

unsigned long timeNow = millis();                    // the time at the start of the loop()
unsigned long pidLastTime = millis();                // when the PID was calculated last
unsigned long joypadLastTime = millis();             // the time the bot last received a joypad command
unsigned long offLineLastTime = millis();            // lasst time the bot came off the line
unsigned long offTheLineTime = 0;                    // how long has the bot been off the line, total since last leaving the line

////// Outputs
// speeds are  -255 (fullspeed back) ..  255 (full speed forward)
int cruiseSpeed = 120;      // default cruise speed when linw following
int speedA = 0;             // viewed from behind motor 'A' is on the left
int speedB = 0;             // viewed from behind motor 'B' is on the right



//////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino functions

void setup() {
  Serial.begin(9600);            // USB serial port (debugging)
  Serial1.begin(9600);           // Data from the RFDuino is read from here

  // Motor pind
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
  pinMode(pinMODE, OUTPUT);
  digitalWrite(pinMODE, HIGH);   //to set controller to Phase/Enable mode

  // Headlights...
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
}


void loop() {
  timeNow = millis();

  readSerial();
  readIRSensors();
  updateLineFollowingStatus();

  if (isLineFollowingMode) {
    calculatePID();
    joypadLineFollowingControlMode();
  } else {     // in manual mode
    joypadManualControlMode();
  }
  motorSpeed(speedA, speedB);
  printVals();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// PID

void calculatePID() {
  // Calculate PID on a regular time basis, return if called too soo
  if ((timeNow - pidLastTime) < PID_SAMPLE_TIME ) {
    return;
  }
  pidLastTime = timeNow;

  // process IR readings via PID
  error_last = error;                                   // store previous error before new one is caluclated
  error = IRvals[0] - IRvals[2];                        // TODO: change to lineFOllowingLib.getIRreading(IR_SENSORID);

  P_error = error * Kp / 10.0;                               // calculate proportional term
  D_error = (error - error_last) * Kd / 10.0;                // calculate differential term
  correction = P_error + D_error;
  speedA = cruiseSpeed + correction;
  speedB = cruiseSpeed - correction;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Joystick Handling

// This is called when the bot is on the line
// speedA and speedB will have already been set by pid_calculate() before this is run
void  joypadLineFollowingControlMode() {
  speedA = speedA + (yAxisValue / 3); //superpose yAxis with PID output speed
  speedB = speedB + (yAxisValue / 3);
}

// This is called when the bot is in manual mode.
// Use it to map the joystick X & Y axis (which both range from -255..255) to motor speeds (also -255..255)

void joypadManualControlMode() {
  // check if we have recent joystick input
  if ( (timeNow - joypadLastTime) > MANUAL_MODE_RADIOSILENCE_TIMEOUT) {
    // no command has been received in the last X millis, err on the side of caution and stop!
    speedA = speedB =  0;
    xAxisValue = yAxisValue = 0;
    return;
  }
  if ( abs(xAxisValue) < JOYPAD_AXIS_DEADZONE)
    xAxisValue = 0;
  if ( abs(yAxisValue) < JOYPAD_AXIS_DEADZONE)
    yAxisValue = 0;

  speedA =  (yAxisValue + xAxisValue) / 4;
  speedB =  (yAxisValue - xAxisValue) / 4;

  if (buttonPressed)
    forceManualMode = 1;
  else
    forceManualMode = 0;

}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Inputs

void readIRSensors() {
  IRvals[0] = analogRead(IR1) + IR1_BIAS; //left looking from behind
  IRvals[1] = analogRead(IR2) + IR2_BIAS; //centre
  IRvals[2] = analogRead(IR3) + IR3_BIAS; //right
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Outputs

void motorSpeed(int _speedA, int _speedB) {
  _speedA = constrain(_speedA, -255, 255);
  _speedB = constrain(_speedB, -255, 255);
  digitalWrite(pinA1, _speedA >= 0 ? HIGH : LOW) ;
  analogWrite (pinA2, abs(_speedA));
  digitalWrite(pinB1, _speedB >= 0 ? HIGH : LOW);
  analogWrite (pinB2, abs(_speedB));
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

void updateLineFollowingStatus() {
  if ((IRvals[1] <= WHITE_THRESHOLD )) {
    offTheLineTime += timeNow - offLineLastTime;
    offLineLastTime = timeNow;

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

// Serial Input
// We're expecting mesages of 5 bytes in the form:  >>XYB
// Where;
// >> = start marker, as-is
// X = unsinged byte for xAxis:          0 .. 255 mapped to -254 .. 254
// Y = unsinged byte for yAxis:          0 .. 255 mapped to -254 .. 254
// B = unsinged byte for button pressed: 0 = no, 1 = yes
void readSerial() {
  static int c = 0, lastChar = 0;
  while (Serial1.available() >= 5) {
    lastChar = c;
    c = Serial1.read();
    if ( ('>' == c) && ('>' == lastChar) ) {
      xAxisValue    = (Serial1.read() - 127) * 2;
      yAxisValue    = (Serial1.read() - 127) * 2;
      buttonPressed = Serial1.read();
      lastChar = c = 0;
      joypadLastTime = timeNow;                      // record the time we last received a joypad command.
    }
  }
}

