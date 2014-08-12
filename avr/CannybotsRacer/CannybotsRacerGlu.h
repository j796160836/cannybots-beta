#ifndef _CANNYBOTSRACERGLU_H
#define _CANNYBOTSRACERGLU_H
#include <CannybotsTypes.h>

// This file is shared with Arduino sketche and the client platforms such as: iOS, Android, Python etc.

#define CFG_ID             "CBLF"
#define CFG_BASE           0
#define LF_MAJOR_VERSION   0
#define LF_MINOR_VERSION   1

//maybe TODO: move to Cannybots lib
//uint32_t  cb_bot_type = 0xCB1F0001;
//uint16_t  cb_version  = LF_MAJOR_VERSION*255 + LF_MINOR_VERSION;
//uint32_t  cb_bot_id   = 0x0000CB01; 


// TODO: move to EEPROM config
#define NUM_MOTORS      2
#define NUM_IR_SENSORS  3
#define STATUS_LED      13
#define MANUAL_MODE_RADIOSILENCE_TIMEOUT 500
#define IR_EMIT_VALUES_INTERVAL 2000


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
CB_ID(10, RACER_TANKCONTROL_MODE, "isTankControlMode");
CB_ID(11, RACER_FORCEMANUAL_MODE, "forceManualMode");

//CB_ID(10, RACER_MOVE, "move");
//CB_ID(11, RACER_TEST, "test");

CB_ID(30, LAPCOUNTER_GETREADY, "getReady");
CB_ID(31, LAPCOUNTER_LAPTIME,  "lapTime");
CB_ID(32, LAPCOUNTER_LAPCOUNT, "lapCount");
CB_ID(32, LAPCOUNTER_STOP,     "lapStop");

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

// The struct members needs a prefix for namespacing; these end up as global static constants and 'stringinfied' versions of variable names are used in the iOS/Android scripting side as-is
// A C/C++ structs order will not be changed by the compiler so the member offset of a cfg value lower down the list will have a unique higher address (no unions!)
// the offset is stored in a 8bit variable, so don't let it go over 255! (e.g. supports 128 ints, more than enough?)
// 1024 EEPROM = settings for 4 different bots types or 4 versions of config
typedef struct __attribute__((packed)) {
  // standard 16 byte header - must be present in this order
  uint32_t  cfg_type;                          // 0   -  4 bytes intended to be ASCII (human readable, e.g. 0xCB1F0001  = CannyBots LineFollowing # 0001)
  uint16_t  cfg_id;                            // 6   -  user defined 32bit identifier
  uint16_t  cfg_version;                       // 4   -  major/minor version
  uint32_t  cfg_authentication_pin;            // 8   -  32 bit / 8 digit pin used for authentication 
  uint32_t  _offsetPadding;                    // 12  -  reserved space for future use, allows us to add up to 4 bytes without upsetting user offsets.

  // user defined config:
  
  bool      cfg_battery_hasSense;              // 16
  uint8_t   cfg_battery_pin_sense;             // 17

  uint16_t  cfg_ir_max;                        // 18
  uint16_t  cfg_ir_whiteThreshold;             // 20
  uint8_t   cfg_ir_pin_1;                      // 22
  uint8_t   cfg_ir_pin_2;                      // 23
  uint8_t   cfg_ir_pin_3;                      // 24
  int16_t   cfg_ir_bias_1;                     // 25
  int16_t   cfg_ir_bias_2;                     // 27
  int16_t   cfg_ir_bias_3;                     // 29

  uint8_t   cfg_motorDriver_type;              // 31
  uint8_t   cfg_motorDriver_driveModePin;      // 32
  uint8_t   cfg_motorDriver_maxSpeed;          // 33
  bool      cfg_motorDriver_hasDriveMode;      // 34
  bool      cfg_motorDriver_hasMotorSense;     // 35
  uint8_t   cfg_motorA_pin_1;                  // 36
  uint8_t   cfg_motorA_pin_2;                  // 37
  uint8_t   cfg_motorA_pin_sense;              // 38
  bool      cfg_motorA_postiveSpeedisFwd;      // 39
  uint8_t   cfg_motorA_id;                     // 40   // e.g. 0 = left, 1 = right
  uint8_t   cfg_motorB_pin_1;                  // 41
  uint8_t   cfg_motorB_pin_2;                  // 42
  uint8_t   cfg_motorB_pin_sense;              // 43
  bool      cfg_motorB_postiveSpeedisFwd;      // 44
  uint8_t   cfg_motorB_id;                     // 45  // e.g. 0 = left, 1 = right
  uint16_t  cfg_motor_speedSmoothingDivisions; // 46  // in manual mode: number of divisions between current and target motor speed
  uint8_t   cfg_motor_speedSmoothingMaxDelta;  // 48  // in manual mode: max motor speed change

  uint16_t  cfg_pid_p;                         // 49
  uint16_t  cfg_pid_i;                         // 51 
  uint16_t  cfg_pid_d;                         // 53
  uint16_t  cfg_pid_divisor;                   // 55
  uint16_t  cfg_pid_sampleTime;                // 57

  uint8_t   cfg_joystick_xAxisDeadzone;        // 59

  uint8_t   cfg_cruiseSpeed_defaultSpeed;      // 60
  uint8_t   cfg_cruiseSpeed_manualMaxSpeed;    // 61

  uint16_t  cfg_offLineMaxTime;                // 62

  uint16_t  cfg_info_printValsInterval;        // 64
  bool      cfg_debugFlag;                     // 66
  
} cb_app_config;  // len = 67

CB_CFG_ID(cfg_type);
CB_CFG_ID(cfg_id);
CB_CFG_ID(cfg_version);
CB_CFG_ID(cfg_authentication_pin);
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
CB_CFG_ID(cfg_motorDriver_driveModePin);
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
CB_CFG_ID(cfg_pid_sampleTime);
CB_CFG_ID(cfg_joystick_xAxisDeadzone);
CB_CFG_ID(cfg_cruiseSpeed_defaultSpeed);
CB_CFG_ID(cfg_cruiseSpeed_manualMaxSpeed);
CB_CFG_ID(cfg_offLineMaxTime);
CB_CFG_ID(cfg_info_printValsInterval);
CB_CFG_ID(cfg_debugFlag);

#ifdef __cplusplus
extern "C" {
#endif
void cannybotsRacerGlu_setup(cb_app_config* settings);  
void cannybotsRacerGlu_setupConfig(cb_app_config* settings);
    
#ifdef __cplusplus
}
#endif

#endif

