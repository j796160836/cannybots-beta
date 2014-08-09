//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Common LineFollowing functoins

// TODO: to be moved to a library


bool isLineFollowingMode = true;
unsigned long pidLastTime = millis();


unsigned long offTheLineTime = 0;
unsigned long offLineLastTime = millis();

// some counters
volatile unsigned long lastCommandTime = millis();
volatile unsigned long loopNowTime = millis();
volatile unsigned long loopLastTime = millis();
volatile unsigned long loopDeltaTime = millis();


void lineFollowing_setup() {
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

}


void lineFollowing_loop() {
    // do some stats...  
  loopNowTime = millis();
  loopDeltaTime = loopNowTime - loopLastTime;
  loopLastTime = loopNowTime;
  
  // read IR sensor values
  read_ir_sensors();
  // publish IR values
  lf_emitIRValues(IRvals[0], IRvals[1], IRvals[2]);
   // count up the time spent off the line, rahter than switching to manual mode the instance the mid sensor goes of the line
  if ((IRvals[1] <= WHITE_THRESHOLD )) {
    offTheLineTime += loopNowTime - offLineLastTime;
    offLineLastTime = loopNowTime;
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

  lf_report_followingMode(isLineFollowingMode);

  if (isLineFollowingMode) {
      if ((loopNowTime - pidLastTime) > PID_SAMPLE_TIME){
        pidLastTime= loopNowTime;
        calculate_PID();
     }
    speedA = speedA + (yAxisValue/3); //superpose yAxis with PID output speed
    speedB = speedB + (yAxisValue/3);
  } else {
    // in manual mode
    if ( (millis() - cb.getLastInboundCommandTime()) > MANUAL_MODE_RADIOSILENCE_TIMEOUT) {
      // no command has been received in the last X millis, err on the side of caution and stop!
      speedA = speedB =  0;
    } else {
      // just allow revese move to get back to line.. 
      if (yAxisValue < 0){
        speedA = yAxisValue/5; //-xAxisValue
        speedB = yAxisValue/5; //xAxisValue
      } else {
        speedA = speedB = 0;
      }
    }
  }
  motor(speedA, speedB);
  delay(5);
  printvalues();

}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors and actuators

// TODO: move to a CannyBots line following library


// read the IR sensors:
// set limit on reading. The reading can be very high and inaccurate on pitch black
void read_ir_sensors() {
  //analogRead(IR1); delay(ANALOG_READING_DELAY);
  IRvals[0] = constrain(analogRead(IR1) - IRbias[0], 0, IR_MAX); //left looking from behind
  //analogRead(IR2); delay(ANALOG_READING_DELAY);
  IRvals[1] = constrain(analogRead(IR2) - IRbias[1], 0, IR_MAX); //centre
  //analogRead(IR3); delay(ANALOG_READING_DELAY);
  IRvals[2] = constrain(analogRead(IR3) - IRbias[2], 0, IR_MAX); //right
  
  //delay (100);
  //CB_DBG("%d,%d,%d, A=%d,%d,%d, IRB(%d,%d,%d), IRMAX=%d, WTHR=%d", IR1, IR2, IR3, A6, A8, A11, IRbias[0],IRbias[1], IRbias[2], IR_MAX, WHITE_THRESHOLD);
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
    cb.callMethod(&RACER_LINEFOLLOWING_MODE, isLineMode);
    lastCall = millis();
  }
}



void printvals_PID() {
  CB_DBG(    "%lu(%lu): IR(%u,%u,%u),Kpd(%d,%d)/100,Sab(%d,%d), XY(%d,%d),MEM(%d)\n",
             loopNowTime,
             loopDeltaTime,
             IRvals[0], IRvals[1], IRvals[2],
             Kp*100, Kd*100, 
             speedA, speedB,
             xAxisValue, yAxisValue,
             cb.getFreeMemory()
        );
   //cb.dumpConfig();
}


// default getter/setters


void setPID_P(int v) {
  Kp = v;
}
void setPID_I(int v) {
  Ki = v;
}
void setPID_D(int v) {
  Kd = v;
}
int getPID_P() {
  return Kp;
}
int getPID_I() {
  return Ki;
}
int getPID_D() {
  return Kd;
}






