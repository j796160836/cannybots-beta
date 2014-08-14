//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots glulogic
#include "CannybotsRacerGlu.h"
#include <Cannybots.h>

void cannybotsRacerGlu_setup(cb_app_config* settings) {
  //TODO:Cannybots::getInstance().setConfigStorage(CFG_ID, CFG_BASE, LF_MAJOR_VERSION, LF_MINOR_VERSION, settings, sizeof(settings));
  Cannybots::getInstance().setConfigStorage(CFG_ID, CFG_BASE, sizeof(cb_app_config), LF_MAJOR_VERSION, LF_MINOR_VERSION);
  cannybotsRacerGlu_setupConfig(settings);
  Cannybots::getInstance().populateVariablesFromConfig();

  // set some sane defaults
  if (settings->cfg_info_printValsInterval < 200) {
    settings->cfg_info_printValsInterval = 200;
  }
}

// Stored Settings  (EEPROM/Flash)
// these registrations are in a function without reference to 'local' storage so we can call this from Arduino, iOS, Android etc
void cannybotsRacerGlu_setupConfig(cb_app_config* settings) {
  CB_REGISTER_CONFIG(cfg_type);
  CB_REGISTER_CONFIG(cfg_id);
  CB_REGISTER_CONFIG(cfg_version);
  CB_REGISTER_CONFIG(cfg_authentication_pin);
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
}

void cannybotsRacerGlu_setupConfigDefaults(cb_app_config* settings) {
  settings->cfg_type = 0;
  settings->cfg_id = 0;
  settings->cfg_version = 0;
  settings->cfg_authentication_pin = 0;
  settings->cfg_battery_hasSense = 0;
  settings->cfg_battery_pin_sense = 0;
  settings->cfg_ir_max = 0;
  settings->cfg_ir_whiteThreshold = 0;
  settings->cfg_ir_pin_1 = 0;
  settings->cfg_ir_pin_2 = 0;
  settings->cfg_ir_pin_3 = 0;
  settings->cfg_ir_bias_1 = 0;
  settings->cfg_ir_bias_2 = 0;
  settings->cfg_ir_bias_3 = 0;
  settings->cfg_motorDriver_type = 0;
  settings->cfg_motorDriver_driveModePin = 0;
  settings->cfg_motorDriver_maxSpeed = 0;
  settings->cfg_motorDriver_hasDriveMode = 0;
  settings->cfg_motorDriver_hasMotorSense = 0;
  settings->cfg_motorA_pin_1 = 0;
  settings->cfg_motorA_pin_2 = 0;
  settings->cfg_motorA_pin_sense = 0;
  settings->cfg_motorA_postiveSpeedisFwd = 0;
  settings->cfg_motorA_id = 0;
  settings->cfg_motorB_pin_1 = 0;
  settings->cfg_motorB_pin_2 = 0;
  settings->cfg_motorB_pin_sense = 0;
  settings->cfg_motorB_postiveSpeedisFwd = 0;
  settings->cfg_motorB_id = 0;
  settings->cfg_motor_speedSmoothingDivisions = 0;
  settings->cfg_motor_speedSmoothingMaxDelta = 0;
  settings->cfg_pid_p = 0;
  settings->cfg_pid_i = 0;
  settings->cfg_pid_d = 0;
  settings->cfg_pid_divisor = 0;
  settings->cfg_pid_sampleTime = 0;
  settings->cfg_joystick_xAxisDeadzone = 0;
  settings->cfg_cruiseSpeed_defaultSpeed = 0;
  settings->cfg_cruiseSpeed_manualMaxSpeed = 0;
  settings->cfg_offLineMaxTime = 0;                            // max numebr of milliseconds to wait before stopping after comin
  settings->cfg_info_printValsInterval = 0;
  Cannybots::getInstance().populateConfigFromVariables();
}

