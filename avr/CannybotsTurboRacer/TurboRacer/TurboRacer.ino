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
#define IR_WHITE_THRESHOLD         800

#define MOTOR_MAX_SPEED            255
#define MOTOR_CRUISE_SPEED         150

#define OFF_LINE_MAX_TIME          0


#define PID_P                      105
#define PID_D                      145

#define PID_SCALE                100.0
#define PID_SAMPLE_TIME              5

#define JOYPAD_ID                    0
#define JOYPAD_AXIS_DEADZONE        20
#define JOYPAD_CONNECTION_TIMEOUT  2000



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
  motorSpeed(0, 0);

  // Headlights...
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
}


void loop() {
  //Serial.println(millis()-timeNow); // loop time delta
  timeNow = millis();
  readSerial();
  readIRSensors();
  printVals();
  updateLineFollowingStatus();

  if (isLineFollowingMode) {
    calculatePID();
    joypadLineFollowingControlMode();
  } else {
    // in manual mode
    if (!forceManualMode) {
      autoFindLine();
    }
    joypadManualControlMode();
  }
  motorSpeed(speedA, -speedB);
}

#define AUTOFIND_SPEED_DELTA 25
#define AUTOFIND_DELAY        5
#define AUTOFIND_MAX_TIME   100

void autoFindLine() {
  return;
  if (offTheLineTime > AUTOFIND_MAX_TIME)
    return;

  // is the left side of the bot on white
  if ((IRvals[0] <= IR_WHITE_THRESHOLD )) {
    speedA = speedA + AUTOFIND_SPEED_DELTA;
    speedB = speedB - AUTOFIND_SPEED_DELTA;
  } else {
    speedA = speedA - AUTOFIND_SPEED_DELTA;
    speedB = speedB + AUTOFIND_SPEED_DELTA;
  }
  //delay(AUTOFIND_DELAY);
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
    int y = yAxisValue;
    int x = xAxisValue;

#define X_DEAD_MIN 64
#define Y_DEAD_MIN 64
#define X_MAX      64

    if (y > 0) {
      if ( y > Y_DEAD_MIN)
        y = map(y, Y_DEAD_MIN, 255, 0, 255);
    } else {
      if ( y < -Y_DEAD_MIN)
        y = map(y, -Y_DEAD_MIN, -255, 0, -255);
    }

    if (x > 0) {
      if ( x > X_DEAD_MIN)
        x = map(x, X_DEAD_MIN, 255, 0, X_MAX);
    } else {
      if ( x < -X_DEAD_MIN)
        x = map(x, -X_DEAD_MIN, -255, 0, -X_MAX);
    }
    // joypad
    //speedA =  y - x;
    //speedB =  y + x;
    // phone
    
    speedA =  yAxisValue/2 - xAxisValue/2;
    speedB =  yAxisValue/2 + xAxisValue/2;
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
// We're expecting messages of 6 bytes in the form:  >>IXYB
// Where;
// >> = start marker, as-is
// I = joypad ID  (0-7 for GZLL joypad ID's)
// X = unsigned byte for xAxis:          0 .. 255 mapped to -254 .. 254
// Y = unsigned byte for yAxis:          0 .. 255 mapped to -254 .. 254
// B = unsigned byte for button pressed: 0 = no, 1 = yes
void readSerial() {
  static int c = 0, lastChar = 0;
  while (Serial1.available() >= ) { FIX this for GZLL/BLE
    lastChar = c;
    c = getCh();
    if ( ('>' == c) && ('>' == lastChar) ) {
      //if (JOYPAD_ID == Serial1.read()) {
      //}
      getCh(); // ignore joypad id
      xAxisValue    = getCh();
      yAxisValue    = getCh();
      buttonPressed = getCh();
      xAxisValue = map(xAxisValue, 0, 255, -255, 255);
      yAxisValue = map(yAxisValue, 0, 255, -255, 255);
      lastChar = c = 0;
      joypadLastTime = timeNow;                      // record the time we last received a joypad command.
    } else if ( ('$' == c) && ('$' == lastChar) ){
      char   varName[6]    = {0};                    // 5  + 1 null

      byte id =  getCh();
      int  bytesRemaining = 20-6;          // we have to consume any unused serial data of the fixed 20 bytes
      
      if (1) { //id == JOYPAD_ID) {
        // read bytes 1..5 = Var name
        for (int i = 0 ; i < 5; i++) {
          varName[i] = getCh();
        }
        varName[5] = 0;
        updateVariable(varName, &bytesRemaining);
#if 1
        Serial.print("id=");
        Serial.print(id);
        Serial.print(",name=");
        Serial.println(varName);
#endif
      }
      // consume and ignore any remaining bytes
      while (bytesRemaining--) {
        getCh();
      }
    }
  }
}

byte getCh() {
  int c=-1;
  //while (!Serial1.available());
  do {
    c= Serial1.read();    
  } while (c==-1);
  //Serial.println(c,HEX);
  return c & 0xFF;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Message Handling

//// Sending variable update out

#define IR_SEND_INTERVAL           50
bool sendIR     = false;
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



// Reading data

int readInt(int* count) {
  int v1 = getCh();
  int v2 = getCh();   
  *count-=2;
  return  (v1<<8) | (v2 & 0xFF);
}



// Writing data

#define SERIAL_MESSAGE_BUFFER_SIZE 23             // 20 byte paload + 2 byte start marker (>>) + 1 null terminator used during string creation (not sent over serial)

void writeData(const char* name, int16_t p1) {
  char msg[SERIAL_MESSAGE_BUFFER_SIZE] = {0}; 
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c%c", 0, name, highByte(p1), lowByte(p1));
  Serial1.write(msg,sizeof(msg)-1);
}

void writeData(const char* name, int16_t p1,  int16_t p2) {
  char msg[SERIAL_MESSAGE_BUFFER_SIZE] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c%c%c%c", 0, name, highByte(p1), lowByte(p1), highByte(p1), lowByte(p2));
  Serial1.write(msg,sizeof(msg)-1);
}

void writeData(const char* name, int16_t p1,  int16_t p2,  int16_t p3) {
  char msg[SERIAL_MESSAGE_BUFFER_SIZE] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c%c%c%c%c%c", 0, name, highByte(p1), lowByte(p1), highByte(p1), lowByte(p2), highByte(p3), lowByte(p3));
  Serial1.write(msg,sizeof(msg)-1);
}



