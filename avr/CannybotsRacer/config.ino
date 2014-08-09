
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots glulogic

void mycannybots_setup() {
  cb.registerHandler(&RACER_CRUISESPEED, lf_updateMotorSpeeds);
  cb.registerHandler(&RACER_LINEFOLLOWING_MODE, lf_updateLineFollowingMode);
  cb.registerHandler(&RACER_PID, lf_updatePID);
  cb.registerHandler(&RACER_IRBIAS, lf_updateBias);
  cb.registerHandler(&RACER_JOYAXIS, lf_updateAxis);
  cb.registerHandler(&RACER_CONFIG, lf_emitConfig);
  cb.registerHandler(&RACER_IRVALS, lf_emitIRValues);
  cb.registerHandler(&RACER_PING, lf_ping);

  // Stored Settings  (EEPROM/Flash)
  cb.setConfigStorage(CFG_ID, CFG_BASE, sizeof(cb_app_config), LF_MAJOR_VERSION, LF_MINOR_VERSION);
  cb.registerConfigParameter(&cfg_bot_type, &cb_bot_type);
  cb.registerConfigParameter(&cfg_version, &cb_version);
  cb.registerConfigParameter(&cfg_bot_id, &cb_bot_id);
  cb.registerConfigParameter(&cfg_battery_hasSense, &hasBattSense);
  cb.registerConfigParameter(&cfg_battery_pin_sense, &BATTERY_PIN);
  cb.registerConfigParameter(&cfg_ir_max, &IR_MAX);
  cb.registerConfigParameter(&cfg_ir_whiteThreshold, &WHITE_THRESHOLD);
  cb.registerConfigParameter(&cfg_ir_pin_1, &IR1);
  cb.registerConfigParameter(&cfg_ir_pin_2, &IR2);
  cb.registerConfigParameter(&cfg_ir_pin_3, &IR3);
  cb.registerConfigParameter(&cfg_ir_bias_1, &IRbias[0]);
  cb.registerConfigParameter(&cfg_ir_bias_2, &IRbias[1]);
  cb.registerConfigParameter(&cfg_ir_bias_3, &IRbias[2]);
  cb.registerConfigParameter(&cfg_motorDriver_type, &motorDriverType);
  cb.registerConfigParameter(&cfg_motorDriver_driveModePin, &pin_MODE);
  cb.registerConfigParameter(&cfg_motorDriver_maxSpeed, &MOTOR_MAX_SPEED);
  cb.registerConfigParameter(&cfg_motorDriver_hasDriveMode, &motorDriverHasMode);
  cb.registerConfigParameter(&cfg_motorDriver_hasMotorSense, &motorDriverHasSense);
  cb.registerConfigParameter(&cfg_motorA_pin_1, &pinA1);
  cb.registerConfigParameter(&cfg_motorA_pin_2, &pinA2);
  cb.registerConfigParameter(&cfg_motorA_pin_sense, &pinAsense);
  cb.registerConfigParameter(&cfg_motorA_postiveSpeedisFwd, &MOTOR_A_POS_IS_FORWARD);
  cb.registerConfigParameter(&cfg_motorA_id, &motorA_id);
  cb.registerConfigParameter(&cfg_motorB_pin_1, &pinB1);
  cb.registerConfigParameter(&cfg_motorB_pin_2, &pinB2);
  cb.registerConfigParameter(&cfg_motorB_pin_sense, &pinBsense);
  cb.registerConfigParameter(&cfg_motorB_postiveSpeedisFwd, &MOTOR_B_POS_IS_FORWARD);
  cb.registerConfigParameter(&cfg_motorB_id, &motorB_id);
  cb.registerConfigParameter(&cfg_motor_speedSmoothingDivisions, &MAX_MOTOR_DELTA_DIVISOR);
  cb.registerConfigParameter(&cfg_motor_speedSmoothingMaxDelta, &MAX_MOTOR_DELTA);
  cb.registerConfigParameter(&cfg_pid_p, &Kp);
  cb.registerConfigParameter(&cfg_pid_i, &Ki);
  cb.registerConfigParameter(&cfg_pid_d, &Kd);
  cb.registerConfigParameter(&cfg_joystick_xAxisDeadzone, &XAXIS_DEADZONE);
  cb.registerConfigParameter(&cfg_cruiseSpeed_defaultSpeed, &baseCruiseSpeed);
  cb.populateVariablesFromConfig();
  cb.begin();
}





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
//#define BOT_TYPE_CUSTOM_PCB 1¨¨
//#define IR_MAX 1000
//#define WHITE_THRESHOLD 700






