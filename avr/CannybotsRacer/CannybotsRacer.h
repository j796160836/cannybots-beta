#ifndef _CANNYBOTSRACER_H
#define _CANNYBOTSRACER_H
#include <CannybotsTypes.h>



// Methods
CB_ID(1, RACER_IRBIAS, "IRbias");
CB_ID(2, RACER_CRUISESPEED, "baseCruiseSpeed");
CB_ID(3, RACER_CURRENTCRUISESPEED, "currentCruiseSpeed");
CB_ID(4, RACER_LINEFOLLOWING_MODE, "isLineFollowingMode");
CB_ID(5, RACER_JOYAXIS, "axis");
CB_ID(6, RACER_PID, "PID");
CB_ID(7, RACER_CONFIG, "config");
CB_ID(8, RACER_IRVALS, "IRvals");
CB_ID(9, RACER_PING, "ping");
//CB_ID(10, RACER_MOVE, "move");
//CB_ID(11, RACER_TEST, "test");

// constants for 'move' method
#define CANNYBOTSRACER_MOVE_STOP     0
#define CANNYBOTSRACER_MOVE_GO       1
#define CANNYBOTSRACER_MOVE_LEFT     2
#define CANNYBOTSRACER_MOVE_RIGHT    3
#define CANNYBOTSRACER_MOVE_SHUFFLE  4

// constants for 'test' method

#define CANNYBOTSRACER_TEST_MOTORS  1
#define CANNYBOTSRACER_TEST_IR_1    2


////////////////////////////////////////////
//
// Configuration  (stored in EEPROM)


#define CFG_ID                           "CBLF"
#define CFG_BASE                         64

#define LF_MAJOR_VERSION 0
#define LF_MINOR_VERSION 1


// The struct needs a prefix because these end up as global static constants and 'stringinfied' versions on variable names used on the iOS/Android scripting side as-is
// A C/C++ structs order will not be changed by the compiler so the member offset of a cfg value lower down the list will have a unique higher address (no unions!)
// the offset is stored in a 8bit variable, so don't let it go over 255! (supports 128 ints, more than enough?)

typedef struct {
  uint32_t  cfg_bot_type;                           // 4 bytes intended to be ASCII (human readable)
  uint16_t  cfg_version;
  uint16_t  cfg_bot_id;
  uint32_t  cfg_authentication_pin;
  
  uint32_t  _offsetPadding;
  
  bool      cfg_battery_hasSense;
  uint8_t   cfg_battery_pin_sense;

  uint16_t  cfg_ir_max;
  uint16_t  cfg_ir_whiteThreshold;
  uint8_t   cfg_ir_pin_1;
  uint8_t   cfg_ir_pin_2;
  uint8_t   cfg_ir_pin_3;
  int16_t   cfg_ir_bias_1;  
  int16_t   cfg_ir_bias_2;  
  int16_t   cfg_ir_bias_3;  

  uint8_t   cfg_motorDriver_type;
  uint8_t   cfg_motorDriver_mode;
  uint8_t   cfg_motorDriver_maxSpeed;
  bool      cfg_motorDriver_hasDriveMode;
  bool      cfg_motorDriver_hasMotorSense;
  uint8_t   cfg_motorA_pin_1;
  uint8_t   cfg_motorA_pin_2;
  uint8_t   cfg_motorA_pin_sense;
  bool      cfg_motorA_postiveSpeedisFwd;
  uint8_t   cfg_motorA_id;                        // e.g. 0 = left, 1 = right
  uint8_t   cfg_motorB_pin_1;
  uint8_t   cfg_motorB_pin_2;
  uint8_t   cfg_motorB_pin_sense;
  bool      cfg_motorB_postiveSpeedisFwd;
  uint8_t   cfg_motorB_id;                        // e.g. 0 = left, 1 = right
  uint16_t  cfg_motor_speedSmoothingDivisions;
  uint8_t   cfg_motor_speedSmoothingMaxDelta;

  uint16_t  cfg_pid_p;
  uint16_t  cfg_pid_i;
  uint16_t  cfg_pid_d;
  uint16_t  cfg_pid_divisor;

  uint8_t   cfg_joystick_xAxisDeadzone;

  uint8_t   cfg_cruiseSpeed_defaultSpeed;
  uint8_t   cfg_cruiseSpeed_manualMaxSpeed;

  uint16_t  cfg_offLineMaxTime;  

  uint16_t  cfg_info_printValsInterval;  
  bool      cfg_debugFlag;
  
} cb_app_config;



CB_CFG_ID(cfg_bot_type);
CB_CFG_ID(cfg_version);
CB_CFG_ID(cfg_bot_id);
CB_CFG_ID(cfg_battery_hasSense);
CB_CFG_ID(cfg_battery_pin_sense);
CB_CFG_ID(cfg_ir_max);
CB_CFG_ID(cfg_ir_whiteThreshold);
CB_CFG_ID(cfg_ir_pin_1);
CB_CFG_ID(cfg_ir_pin_2);
CB_CFG_ID(cfg_ir_pin_3);
CB_CFG_ID(cfg_ir_bias_1);
CB_CFG_ID(cfg_ir_bias_2);
CB_CFG_ID(cfg_ir_bias_3);
CB_CFG_ID(cfg_motorDriver_type);
CB_CFG_ID(cfg_motorDriver_mode);
CB_CFG_ID(cfg_motorDriver_maxSpeed);
CB_CFG_ID(cfg_motorDriver_hasDriveMode);
CB_CFG_ID(cfg_motorDriver_hasMotorSense);
CB_CFG_ID(cfg_motorA_pin_1);
CB_CFG_ID(cfg_motorA_pin_2);
CB_CFG_ID(cfg_motorA_pin_sense);
CB_CFG_ID(cfg_motorA_postiveSpeedisFwd);
CB_CFG_ID(cfg_motorA_id);
CB_CFG_ID(cfg_motorB_pin_1);
CB_CFG_ID(cfg_motorB_pin_2);
CB_CFG_ID(cfg_motorB_pin_sense);
CB_CFG_ID(cfg_motorB_postiveSpeedisFwd);
CB_CFG_ID(cfg_motorB_id);
CB_CFG_ID(cfg_motor_speedSmoothingDivisions);
CB_CFG_ID(cfg_motor_speedSmoothingMaxDelta);
CB_CFG_ID(cfg_pid_p);
CB_CFG_ID(cfg_pid_i);
CB_CFG_ID(cfg_pid_d);
CB_CFG_ID(cfg_pid_divisor);
CB_CFG_ID(cfg_joystick_xAxisDeadzone);
CB_CFG_ID(cfg_cruiseSpeed_defaultSpeed);
CB_CFG_ID(cfg_cruiseSpeed_manualMaxSpeed);
CB_CFG_ID(cfg_offLineMaxTime);
CB_CFG_ID(cfg_info_printValsInterval);
CB_CFG_ID(cfg_debugFlag);

#endif

