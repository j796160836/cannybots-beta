//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Common LineFollowing functoins

// TODO: to be moved to a library


unsigned long pidLastTime = millis();


unsigned long offTheLineTime = 0;
unsigned long offLineLastTime = millis();

// some counters
volatile unsigned long lastCommandTime = millis();
volatile unsigned long loopNowTime = millis();
volatile unsigned long loopLastTime = millis();
volatile unsigned long loopDeltaTime = millis();


void lineFollowing_setup() {
  pinMode(settings.cfg_motorA_pin_1, OUTPUT);
  pinMode(settings.cfg_motorA_pin_2, OUTPUT);
  pinMode(settings.cfg_motorB_pin_1, OUTPUT);
  pinMode(settings.cfg_motorB_pin_2, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

#ifdef BOT_TYPE_CUSTOM_PCB
  pinMode(settings.cfg_motorDriver_driveModePin, OUTPUT);
  digitalWrite(settings.cfg_motorDriver_driveModePin, HIGH); //to set controller to Phase/Enable mode
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
  if ((IRvals[1] <= settings.cfg_ir_whiteThreshold )) {
    offTheLineTime += loopNowTime - offLineLastTime;
    offLineLastTime = loopNowTime;
    if (offTheLineTime > settings.cfg_offLineMaxTime) {
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
      if ((loopNowTime - pidLastTime) > settings.cfg_pid_sampleTime){
        pidLastTime= loopNowTime;
        pid_calculate();
     }
     joystick_lineFollowingControlMode();
  } else {
    // in manual mode
    if ( (millis() - cb.getLastInboundCommandTime()) > MANUAL_MODE_RADIOSILENCE_TIMEOUT) {
      // no command has been received in the last X millis, err on the side of caution and stop!
      speedA = speedB =  0;
    } else {
      joystick_manualControlMode();
    }
  }
  
  //
  speedA = constrain( settings.cfg_motorB_postiveSpeedisFwd?speedA:-speedA, -settings.cfg_motorDriver_maxSpeed, settings.cfg_motorDriver_maxSpeed);
  speedB = constrain( settings.cfg_motorB_postiveSpeedisFwd?speedB:-speedB, -settings.cfg_motorDriver_maxSpeed, settings.cfg_motorDriver_maxSpeed);

  motor(speedA, speedB);
  //delay(5);
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
  IRvals[0] = constrain(analogRead(settings.cfg_ir_pin_1) - settings.cfg_ir_bias_1, 0, settings.cfg_ir_max); //left looking from behind
  //analogRead(IR2); delay(ANALOG_READING_DELAY);
  IRvals[1] = constrain(analogRead(settings.cfg_ir_pin_2) - settings.cfg_ir_bias_2, 0, settings.cfg_ir_max); //centre
  //analogRead(IR3); delay(ANALOG_READING_DELAY);
  IRvals[2] = constrain(analogRead(settings.cfg_ir_pin_3) - settings.cfg_ir_bias_3, 0, settings.cfg_ir_max); //right
  
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
  _speedA = constrain(_speedA, -settings.cfg_motorDriver_maxSpeed, settings.cfg_motorDriver_maxSpeed);
  _speedB = constrain(_speedB, -settings.cfg_motorDriver_maxSpeed, settings.cfg_motorDriver_maxSpeed);

  digitalWrite(settings.cfg_motorA_pin_1, _speedA >= 0 ? HIGH : LOW) ;
  analogWrite (settings.cfg_motorA_pin_2, abs(_speedA));

  digitalWrite(settings.cfg_motorB_pin_1, _speedB >= 0 ? HIGH : LOW);
  analogWrite (settings.cfg_motorB_pin_2, abs(_speedB));
}


// motor controller function
void motor_ORG(int _speedA, int _speedB) // V4
{
  _speedA = constrain(_speedA, -settings.cfg_motorDriver_maxSpeed, settings.cfg_motorDriver_maxSpeed);
  _speedB = constrain(_speedB, -settings.cfg_motorDriver_maxSpeed, settings.cfg_motorDriver_maxSpeed);
  if (_speedA >= 0) {
    analogWrite(settings.cfg_motorA_pin_1, _speedA);
    analogWrite(settings.cfg_motorA_pin_2, 0);
  } else {
    analogWrite(settings.cfg_motorA_pin_1, 0);
    analogWrite(settings.cfg_motorA_pin_2, abs(_speedA));
  }
  if (_speedB >= 0) {
    analogWrite(settings.cfg_motorB_pin_1, _speedB);
    analogWrite(settings.cfg_motorB_pin_2, 0);
  } else {
    analogWrite(settings.cfg_motorB_pin_1, 0);
    analogWrite(settings.cfg_motorB_pin_2, abs(_speedB));
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
  if ( millis() - lastPrint < settings.cfg_info_printValsInterval )
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
    //lap_started(); 
    //lap_completed();

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


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Remotely called funcs


void lf_updateMotorSpeeds(int _speedA, int _speedB, int _dummy) {
}

void lf_updateAxis(int xAxis, int yAxis, int _dummy) {  
  xAxisValue = xAxis;  //joy X axis vale  = Direction  -255 to 255
  yAxisValue = yAxis;  //joy y axis vale = Throttle    -255 to 255
  //CB_DBG("joy=%d,%d", xAxis,yAxis);

}


void lf_updatePID(int _Kp, int _Ki, int _Kd) {
  //CB_DBG("PID=%d,%d,%d", _Kp, _Ki, _Kd);
  setPID_P(_Kp);
  setPID_D(_Kd); 
  cb.setConfigParameterValue(&cfg_pid_p, &_Kp);
  cb.setConfigParameterValue(&cfg_pid_d, &_Kd);
}

void lf_updateBias (int b1, int b2, int b3) {
  //CB_DBG("Bias=%d,%d,%d", b1, b2, b3);
  // TODO: change to the generic:  cb.setConfigParameterValue(&NV_IRBIAS_1), no need to specify variable address again
  cb.setConfigParameterValue(&cfg_ir_bias_1, &settings.cfg_ir_bias_1);
  cb.setConfigParameterValue(&cfg_ir_bias_2, &settings.cfg_ir_bias_1);
  cb.setConfigParameterValue(&cfg_ir_bias_3, &settings.cfg_ir_bias_1);
}

void lf_updateLineFollowingMode(int _forceManualMode, int _d1, int _d2) {
  //CB_DBG("ForceManual=%d", _forceManualMode);
  forceManualMode = _forceManualMode;
}
void lf_updateTankControlMode(int _isTankControlMode, int _d1, int _d2) {
  isTankControlMode = isTankControlMode;
}



void lf_emitConfig(int _d1, int _d2, int _d3) {
  cb.callMethod(&RACER_PID, getPID_P(), getPID_I(), getPID_D());
  cb.callMethod(&RACER_IRBIAS, settings.cfg_ir_bias_1,settings.cfg_ir_bias_2, settings.cfg_ir_bias_3);
}

void lf_emitIRValues(int v1, int v2, int v3) {
  static unsigned long lastCall = millis();
  if (millis() - lastCall > 200) {
    cb.callMethod(&RACER_IRVALS, v1, v2, v3);
    lastCall = millis();
  }
}


void lf_ping(int v1) {
  //CB_DBG("ping", v1)
}






