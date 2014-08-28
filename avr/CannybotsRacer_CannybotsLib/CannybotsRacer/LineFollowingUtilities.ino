//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Common LineFollowing functoins

// some stats & timers

unsigned long pidLastTime = millis();
unsigned long offTheLineTime = 0;
unsigned long offLineLastTime = millis();
volatile unsigned long lastCommandTime = millis();
volatile unsigned long loopLastTime = millis();


void lineFollowingUtilities_setup() {
  Cannybots& cb = Cannybots::getInstance();
  cb.registerHandler(&RACER_TANKCONTROL_MODE, lf_updateTankControlMode);
  cb.registerHandler(&RACER_FORCEMANUAL_MODE, lf_updateForceManualMode);  
  cb.registerHandler(&RACER_PID, lf_updatePID);
  cb.registerHandler(&RACER_JOYAXIS, lf_updateAxis);
  cb.registerHandler(&RACER_CONFIG, lf_emitConfig);
  cb.registerHandler(&RACER_IRVALS, lf_emitIRValues);
  cb.registerHandler(&RACER_IRBIAS, lf_updateBias);
  cb.registerHandler(&RACER_PING, lf_ping);  
  cannybotsRacerGlu_setup(&settings); 
  cb.begin();
  
  pinMode(settings.cfg_motorA_pin_1, OUTPUT);
  digitalWrite(settings.cfg_motorA_pin_1, LOW);
  pinMode(settings.cfg_motorA_pin_2, OUTPUT);
  digitalWrite(settings.cfg_motorA_pin_2, LOW);
  pinMode(settings.cfg_motorB_pin_1, OUTPUT);
  digitalWrite(settings.cfg_motorB_pin_1, LOW);
  pinMode(settings.cfg_motorB_pin_2, OUTPUT);
  digitalWrite(settings.cfg_motorB_pin_2, LOW);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  if (settings.cfg_motorDriver_hasDriveMode) {
    pinMode(settings.cfg_motorDriver_driveModePin, OUTPUT);
    digitalWrite(settings.cfg_motorDriver_driveModePin, HIGH); //to set controller to Phase/Enable mode
  }
  
  
}


void lineFollowingUtilities_loop() {
    cb.update();

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
      xAxisValue = yAxisValue = 0;
    } else {
      joystick_manualControlMode();
    }
  }
  printvalues();
   
  //CB_DBG("isfwd: %d,%d", settings.cfg_motorA_postiveSpeedisFwd, settings.cfg_motorB_postiveSpeedisFwd);
  
  //speedA = settings.cfg_motorA_postiveSpeedisFwd==1?speedA:-speedA; 
  /*if (settings.cfg_motorA_postiveSpeedisFwd == 1) {
      speedA = speedA;
  } else {
     speedA = -speedA;
  }
  if (settings.cfg_motorB_postiveSpeedisFwd == 1) {
      speedB = speedB;
  } else {
     speedB = -speedB;
  }*/
  //printvalues();


  motor(speedA, -speedB);
  //delay(5);
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
  IRvals[0] = constrain(analogRead(settings.cfg_ir_pin_1) + settings.cfg_ir_bias_1, 0, settings.cfg_ir_max); //left looking from behind
  //analogRead(IR2); delay(ANALOG_READING_DELAY);
  IRvals[1] = constrain(analogRead(settings.cfg_ir_pin_2) + settings.cfg_ir_bias_2, 0, settings.cfg_ir_max); //centre
  //analogRead(IR3); delay(ANALOG_READING_DELAY);
  IRvals[2] = constrain(analogRead(settings.cfg_ir_pin_3) + settings.cfg_ir_bias_3, 0, settings.cfg_ir_max); //right
  
  //delay (100);
  //CB_DBG("%d,%d,%d, A=%d,%d,%d, IRB(%d,%d,%d), IRMAX=%d, WTHR=%d", IR1, IR2, IR3, A6, A8, A11, IRbias[0],IRbias[1], IRbias[2], IR_MAX, WHITE_THRESHOLD);
}

void motor(int _speedA, int _speedB) {
  switch (settings.cfg_motorDriver_type) {
    case 0:   motor_customPCB_v1(_speedA, _speedB);  break;
    case 1:   motor_v0(_speedA, _speedB);  break;
    default:  motor_customPCB_v1(_speedA, _speedB);  break;
  }
}

void motor_customPCB_v1(int _speedA, int _speedB)
{
  _speedA = constrain(_speedA, -settings.cfg_motorDriver_maxSpeed, settings.cfg_motorDriver_maxSpeed);
  _speedB = constrain(_speedB, -settings.cfg_motorDriver_maxSpeed, settings.cfg_motorDriver_maxSpeed);

  //CB_DBG("A=%d,B=%d, %d,%d,%d,%d", _speedA, _speedB, settings.cfg_motorA_pin_1, settings.cfg_motorA_pin_2, settings.cfg_motorB_pin_1, settings.cfg_motorB_pin_2);
  digitalWrite(settings.cfg_motorA_pin_1, _speedA >= 0 ? HIGH : LOW) ;
  analogWrite (settings.cfg_motorA_pin_2, abs(_speedA));

  digitalWrite(settings.cfg_motorB_pin_1, _speedB >= 0 ? HIGH : LOW);
  analogWrite (settings.cfg_motorB_pin_2, abs(_speedB));
}


// motor controller function
void motor_v0(int _speedA, int _speedB) // V4
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
  print_debug();
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

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Remotely called funcs


void lf_updateMotorSpeeds(int _speedA, int _speedB, int _dummy) {
}

void lf_updateAxis(int xAxis, int yAxis, int _dummy) {  
  xAxisValue = xAxis;  //joy X axis vale  e.g. Direction  -255 to 255
  yAxisValue = yAxis;  //joy y axis vale  e.g. Throttle    -255 to 255
  CB_DBG("joy=%d,%d", xAxis,yAxis);
}

//TODO: this can now be replace with tke configParameter calls
void lf_updatePID(int _Kp, int _Ki, int _Kd) {
  //CB_DBG("PID=%d,%d,%d", _Kp, _Ki, _Kd);
  cb.setConfigParameterValue(&cfg_pid_p, &_Kp);
  cb.setConfigParameterValue(&cfg_pid_d, &_Kd);
}

//TODO: this can now be replace with tke configParameter calls
void lf_updateBias (int b1, int b2, int b3) {
  //CB_DBG("Bias=%d,%d,%d", b1, b2, b3);
  cb.setConfigParameterValue(&cfg_ir_bias_1, &b1);
  cb.setConfigParameterValue(&cfg_ir_bias_2, &b2);
  cb.setConfigParameterValue(&cfg_ir_bias_3, &b3);
}


void lf_updateForceManualMode(int _forceManualMode, int _d1, int _d2) {
  CB_DBG("ForceManual=%d", _forceManualMode);
  forceManualMode = _forceManualMode;
}


void lf_updateTankControlMode(int _isTankControlMode, int _d1, int _d2) {
  CB_DBG("TankMode=%d", _isTankControlMode);
  isTankControlMode = _isTankControlMode;
}

void lf_emitConfig(int _d1, int _d2, int _d3) {
  cb.callMethod(&RACER_PID, settings.cfg_pid_p, settings.cfg_pid_i, settings.cfg_pid_d);
  cb.callMethod(&RACER_IRBIAS, settings.cfg_ir_bias_1,settings.cfg_ir_bias_2, settings.cfg_ir_bias_3);
}

void lf_emitIRValues(int v1, int v2, int v3) {
  static unsigned long lastCall = millis();
  if (millis() - lastCall > IR_EMIT_VALUES_INTERVAL) {
    cb.callMethod(&RACER_IRVALS, v1, v2, v3);
    lastCall = millis();
  }
}


void lf_ping(int v1) {
  //CB_DBG("ping", v1)
}



void lineFollowingUtilities_lapStarted(uint32_t time, uint16_t count) {
  cb.callMethod(&LAPCOUNTER_GETREADY, time);   
  cb.callMethod(&LAPCOUNTER_LAPCOUNT, count);
}

void lineFollowingUtilities_lapComplete(uint32_t time, uint16_t count) {
  cb.callMethod(&LAPCOUNTER_LAPTIME, time);
  cb.callMethod(&LAPCOUNTER_LAPCOUNT, count);
}

void   lineFollowingUtilities_stopLapCounting() {
  cb.callMethod(&LAPCOUNTER_STOP, lapCount);
}


/* Debugging bits'n'pieces:

  cb.dumpConfig();  
   
  CB_DBG("s=%ld",settings.cfg_bot_type);
  CB_DBG("s=%d",settings.cfg_version);
  CB_DBG("s=%ld",settings.cfg_bot_id);
  CB_DBG("s=%d",settings.cfg_battery_hasSense);
  CB_DBG("s=%d",settings.cfg_battery_pin_sense);
  CB_DBG("s=%d",settings.cfg_ir_max);
  CB_DBG("s=%d",settings.cfg_ir_whiteThreshold);
  CB_DBG("s=%d",settings.cfg_ir_pin_1);
  CB_DBG("s=%d",settings.cfg_ir_pin_2);
  CB_DBG("s=%d",settings.cfg_ir_pin_3);
  CB_DBG("s=%d",settings.cfg_ir_bias_1);
  CB_DBG("s=%d",settings.cfg_ir_bias_2);
  CB_DBG("s=%d",settings.cfg_ir_bias_3);
  CB_DBG("s=%d",settings.cfg_motorDriver_type);
  CB_DBG("s=%d",settings.cfg_motorDriver_driveModePin);
  CB_DBG("s=%d",settings.cfg_motorDriver_maxSpeed);
  CB_DBG("s=%d",settings.cfg_motorDriver_hasDriveMode);
  CB_DBG("s=%d",settings.cfg_motorDriver_hasMotorSense);
  CB_DBG("s=%d",settings.cfg_motorA_pin_1);
  CB_DBG("s=%d",settings.cfg_motorA_pin_2);
  CB_DBG("s=%d",settings.cfg_motorA_pin_sense);
  CB_DBG("s=%d",settings.cfg_motorA_postiveSpeedisFwd);
  CB_DBG("s=%d",settings.cfg_motorA_id);
  CB_DBG("s=%d",settings.cfg_motorB_pin_1);
  CB_DBG("s=%d",settings.cfg_motorB_pin_2);
  CB_DBG("s=%d",settings.cfg_motorB_pin_sense);
  CB_DBG("s=%d",settings.cfg_motorB_postiveSpeedisFwd);
  CB_DBG("s=%d",settings.cfg_motorB_id);
  CB_DBG("s=%d",settings.cfg_motor_speedSmoothingDivisions);
  CB_DBG("s=%d",settings.cfg_motor_speedSmoothingMaxDelta);
  CB_DBG("s=%d",settings.cfg_pid_p);
  CB_DBG("s=%d",settings.cfg_pid_i);
  CB_DBG("s=%d",settings.cfg_pid_d);
  CB_DBG("s=%d",settings.cfg_pid_divisor);
  CB_DBG("s=%d",settings.cfg_pid_sampleTime);
  CB_DBG("s=%d",settings.cfg_joystick_xAxisDeadzone);
  CB_DBG("s=%d",settings.cfg_cruiseSpeed_defaultSpeed);
  CB_DBG("s=%d",settings.cfg_cruiseSpeed_manualMaxSpeed);
  CB_DBG("s=%d",settings.cfg_offLineMaxTime);
  CB_DBG("s=%d",settings.cfg_info_printValsInterval);
*/


// Various bot wiring configs...

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








