
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots glulogic



void cannybots_setup() {
  cb.registerHandler(&RACER_CRUISESPEED, lf_updateMotorSpeeds);
  cb.registerHandler(&RACER_LINEFOLLOWING_MODE, lf_updateLineFollowingMode);
  cb.registerHandler(&RACER_TANKCONTROL_MODE, lf_updateTankControlMode);
  
  cb.registerHandler(&RACER_PID, lf_updatePID);
  cb.registerHandler(&RACER_IRBIAS, lf_updateBias);
  cb.registerHandler(&RACER_JOYAXIS, lf_updateAxis);
  cb.registerHandler(&RACER_CONFIG, lf_emitConfig);
  cb.registerHandler(&RACER_IRVALS, lf_emitIRValues);
  cb.registerHandler(&RACER_PING, lf_ping);

  // Stored Settings  (EEPROM/Flash)
  cb.setConfigStorage(CFG_ID, CFG_BASE, sizeof(cb_app_config), LF_MAJOR_VERSION, LF_MINOR_VERSION);

  CB_REGISTER_CONFIG(cfg_bot_type);
  CB_REGISTER_CONFIG(cfg_version);
  CB_REGISTER_CONFIG(cfg_bot_id);
  CB_REGISTER_CONFIG(cfg_battery_hasSense);
  CB_REGISTER_CONFIG(cfg_battery_pin_sense);
  CB_REGISTER_CONFIG(cfg_ir_max);
  CB_REGISTER_CONFIG(cfg_ir_whiteThreshold);
  CB_REGISTER_CONFIG(cfg_ir_pin_1);
  CB_REGISTER_CONFIG(cfg_ir_pin_2);
  CB_REGISTER_CONFIG(cfg_ir_pin_3);
  CB_REGISTER_CONFIG(cfg_ir_bias_1);
  CB_REGISTER_CONFIG(cfg_ir_bias_2);
  CB_REGISTER_CONFIG(cfg_ir_bias_3);
  CB_REGISTER_CONFIG(cfg_motorDriver_type);
  CB_REGISTER_CONFIG(cfg_motorDriver_driveModePin);
  CB_REGISTER_CONFIG(cfg_motorDriver_maxSpeed);
  CB_REGISTER_CONFIG(cfg_motorDriver_hasDriveMode);
  CB_REGISTER_CONFIG(cfg_motorDriver_hasMotorSense);
  CB_REGISTER_CONFIG(cfg_motorA_pin_1);
  CB_REGISTER_CONFIG(cfg_motorA_pin_2);
  CB_REGISTER_CONFIG(cfg_motorA_pin_sense);
  CB_REGISTER_CONFIG(cfg_motorA_postiveSpeedisFwd);
  CB_REGISTER_CONFIG(cfg_motorA_id);
  CB_REGISTER_CONFIG(cfg_motorB_pin_1);
  CB_REGISTER_CONFIG(cfg_motorB_pin_2);
  CB_REGISTER_CONFIG(cfg_motorB_pin_sense);
  CB_REGISTER_CONFIG(cfg_motorB_postiveSpeedisFwd);
  CB_REGISTER_CONFIG(cfg_motorB_id);
  CB_REGISTER_CONFIG(cfg_motor_speedSmoothingDivisions);
  CB_REGISTER_CONFIG(cfg_motor_speedSmoothingMaxDelta);
  CB_REGISTER_CONFIG(cfg_pid_p);
  CB_REGISTER_CONFIG(cfg_pid_i);
  CB_REGISTER_CONFIG(cfg_pid_d);
  CB_REGISTER_CONFIG(cfg_pid_divisor);
  CB_REGISTER_CONFIG(cfg_pid_sampleTime);
  CB_REGISTER_CONFIG(cfg_joystick_xAxisDeadzone);
  CB_REGISTER_CONFIG(cfg_cruiseSpeed_defaultSpeed);
  CB_REGISTER_CONFIG(cfg_cruiseSpeed_manualMaxSpeed);
  CB_REGISTER_CONFIG(cfg_offLineMaxTime);
  CB_REGISTER_CONFIG(cfg_info_printValsInterval);
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






