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
#define RACER_ID                     0


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

#define IR1_BIAS                     12
#define IR2_BIAS                     0
#define IR3_BIAS                     -75

#define IR_WHITE_THRESHOLD         700

#define MOTOR_MAX_SPEED            255
#define MOTOR_CRUISE_SPEED         120

#define OFF_LINE_MAX_TIME          0


#define PID_P                      20
#define PID_D                      90

#define PID_SCALE                100.0
#define PID_SAMPLE_TIME              5

#define JOYPAD_AXIS_DEADZONE        20




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

////// Outputs

// speeds are  -255 (fullspeed back) ..  255 (full speed forward)

int cruiseSpeed = MOTOR_CRUISE_SPEED;      // default cruise speed when line following

// The current requested/calculated motor speeds
int speedA = 0;             // viewed from behind motor 'A' is on the left
int speedB = 0;             // viewed from behind motor 'B' is on the right


// Lap Timing
unsigned long currentStartLapTime = millis();
int  lapCount = 0;


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

unsigned long loopTime = 0;
void loop() {
  loopTime = millis() - timeNow;
  timeNow = millis();

  readSerial();
  readIRSensors();
  updateLineFollowingStatus();
  detectLapMarkers();

  if (isLineFollowingMode) {
    calculatePID();
    joypadLineFollowingControlMode();
  } else {
    // in manual mode
    joypadManualControlMode();
  }
  motorSpeed(speedA, -speedB);
  printVals();
}

#define GREY_MIN 650 
#define GREY_MAX 900
void detectLapMarkers() {
  if ( ( IRvals[0] > GREY_MIN ) && (IRvals[0] < GREY_MAX  ) &&
       ( IRvals[1] > GREY_MIN ) && (IRvals[1] < GREY_MAX  ) &&
       ( IRvals[2] > GREY_MIN ) && (IRvals[2] < GREY_MAX  ) ) {
    // debounce (seconds)         
    if (  ( millis()-currentStartLapTime) > 2000 ) {
      Serial.println("Lap marker detected");
      lap_completed();
    //lap_started();
    }
  }
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
  // If the xis readings are small set them to 0
  if ( abs(xAxisValue) < JOYPAD_AXIS_DEADZONE)
    xAxisValue = 0;
  if ( abs(yAxisValue) < JOYPAD_AXIS_DEADZONE)
    yAxisValue = 0;

  speedA =  yAxisValue / 2 - xAxisValue / 2;
  speedB =  yAxisValue / 2 + xAxisValue / 2;
}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Inputs

void readIRSensors() {
  analogRead(IR1_PIN); //delay(5);
  IRvals[0] = analogRead(IR1_PIN) + IR1_BIAS; //left looking from behind
  analogRead(IR2_PIN); //delay(5);
  IRvals[1] = analogRead(IR2_PIN) + IR2_BIAS; //centre
  analogRead(IR3_PIN); //delay(5);
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
  if (IRvals[1] >= IR_WHITE_THRESHOLD ) {
    isLineFollowingMode = 1;
  } else {
    isLineFollowingMode = 0;
  }
  if (buttonPressed) {
    forceManualMode = true;
    isLineFollowingMode = 0;
  }
}

void printVals() {
  static unsigned long lastPrint = millis();
  if (millis() - lastPrint < 1000) {
    return;
  }
  lastPrint = millis();

  Serial.print(timeNow);
  Serial.print(",");
  Serial.print(loopTime);
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
  Serial.print("),mem=");
  Serial.println(freeRam());

  //lap_started();
  //lap_completed();
  //lap_stopTiming();
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Lap Timing

// this should be called at least once to inform the client (e.g. phone) that the bot has started racing
void lap_started() {
  currentStartLapTime = timeNow;
  writeData("LAPGO",  0, lapCount);
  //lapCount++;
}

// call on lap complete, updates phone with current lap time & lap count, it then restarts the lap timer and incremetn the laptcount
void lap_completed() {
  writeData("LAPST",  timeNow - currentStartLapTime, lapCount);
  currentStartLapTime = timeNow;
  lapCount++;
}

// this should be called to tell the client (e.g. phone app) that lap timing and counting has finished
void lap_stopTiming() {
  writeData("LEND");
}




// Serial Input
// We're expecting messages of 4 bytes in the form:  >>ICL
// Where;
// >> = start marker, as-is
// I  = ID
// L  = bytes remaining

void readSerial() {
  static int c = 0, lastChar = 0;
  char   varName[6]  = {0};                    // 5  + 1 null
  if (Serial1.available() >= 2) { // =  2 (>>) + 2 (CRC,ID) + 5 (varname) + 1 (bytesleft)
    //Serial.println("message ready!");

    lastChar = c;
    c = getCh();
    if ( ( ('>' == c) && ('>' == lastChar) ) ) {
      int crc            = getCh();
      int id             = getCh();
      for (int i = 0 ; i < 5; i++) {
        varName[i] = getCh();
      }
      varName[5] = 0;
      int bytesRemaining = getCh();
      //while ( Serial1.available() < bytesRemaining);
      updateVariable(varName, &bytesRemaining);
#if 0
      Serial.print("id=");
      Serial.print(id);
      Serial.print(",name=");
      Serial.println(varName);
#endif
      //Serial.print("slurp...");
      while (bytesRemaining-- > 0) {
        getCh();
      }
      //Serial.println("...ah");
      c = 0;
    }
    // consume and ignore any remaining bytes
  }
}

byte getCh() {
  int c = -1;
  while (!Serial1.available());
  do {
    c = Serial1.read();
    delay(1);
  } while (c == -1);
  //Serial.write(c);
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
  //writeData("IRVAL", IRvals[0], IRvals[1], IRvals[2]);
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
    joypadLastTime = millis();
  } else if (variableNameMatches(name, "PID_P")) {
    Kp = readInt(count);
  } else if (variableNameMatches(name, "PID_D")) {
    Kd = readInt(count);
  } else if (variableNameMatches(name, "GETCF")) {
    //writeData("PID", Kp, Kd);
  } else if (variableNameMatches(name, "SNDIR")) {
    sendIR = readInt(count);
  } else {
    // unrecognised variable name, ignore.
  }
}



// Reading data

int readInt(int* count) {
  byte v1 = getCh();
  byte v2 = getCh();
  *count -= 2;
  return  ( (v1 & 0xFF) << 8) | (v2 & 0xFF);
}



// Writing data

void writeData(const char* name) {
  char msg[9] = {0};
  snprintf(msg, sizeof(msg), ">>%c%c%5.5s%c%c%c", 0, RACER_ID, name, 0);
  Serial1.write(msg, sizeof(msg) - 1);
}

void writeData(const char* name, int16_t p1) {
  char msg[13] = {0};
  snprintf(msg, sizeof(msg), ">>%c%c%5.5s%c%c%c", 0, RACER_ID, name, 2, highByte(p1), lowByte(p1));
  //Serial.print("Write:");
  //Serial.println(  sizeof(msg) - 1,DEC);
  Serial1.write(msg, sizeof(msg) - 1);
  //Serial1.flush();
}

void writeData(const char* name, int16_t p1,  int16_t p2) {
  char msg[15] = {0};
  snprintf(msg, sizeof(msg), ">>%c%c%5.5s%c%c%c%c%c", 0, RACER_ID, name, 4, highByte(p1), lowByte(p1), highByte(p2), lowByte(p2));
  Serial1.write(msg, sizeof(msg) - 1);
}

void writeData(const char* name, int16_t p1,  int16_t p2,  int16_t p3) {
  char msg[17] = {0};
  snprintf(msg, sizeof(msg), ">>%c%c%5.5s%c%c%c%c%c%c%c", 0, RACER_ID, name, 6, highByte(p1), lowByte(p1), highByte(p2), lowByte(p2), highByte(p3), lowByte(p3));
  Serial1.write(msg, sizeof(msg) - 1);
}



