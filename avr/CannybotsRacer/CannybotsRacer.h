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
// max 12 (defined in Cannybots.h)

// constants for 'move'
#define CANNYBOTSRACER_MOVE_STOP     0
#define CANNYBOTSRACER_MOVE_GO       1
#define CANNYBOTSRACER_MOVE_LEFT     2
#define CANNYBOTSRACER_MOVE_RIGHT    3
#define CANNYBOTSRACER_MOVE_SHUFFLE  4

// constants for 'test'

#define CANNYBOTSRACER_TEST_MOTORS  1
#define CANNYBOTSRACER_TEST_IR_1    2




#define NV_ID                           "CBLF"
#define NV_BASE                         64

typedef struct {
  uint16_t  version;
  uint16_t  bot_id;

  bool      battery_hasSense;
  uint8_t   battery_pin_sense;

  uint16_t  ir_max;
  uint16_t  ir_whiteThreshold;
  uint8_t   ir_pin_1;
  uint8_t   ir_pin_2;
  uint8_t   ir_pin_3;
  int16_t   ir_bias_1;  
  int16_t   ir_bias_2;  
  int16_t   ir_bias_3;  

  uint8_t   motorDriver_type;
  uint8_t   motorDriver_mode;
  uint8_t   motorDriver_maxSpeed;
  bool      motorDriver_hasDriveMode;
  bool      motorDriver_hasMotorSense;
  uint8_t   motor1_pin_A;
  uint8_t   motor1_pin_B;
  uint8_t   motor1_pin_sense;
  bool      motor1_postiveSpeedisFwd;
  uint8_t   motor1_id;                        // e.g. 0 = left, 1 = right
  uint8_t   motor2_pin_A;
  uint8_t   motor2_pin_B;
  uint8_t   motor2_pin_sense;
  bool      motor2_postiveSpeedisFwd;
  uint8_t   motor2_id;                        // e.g. 0 = left, 1 = right
  uint16_t  motor_speedSmoothingDivisions;
  uint8_t   motor_speedSmoothingMaxDelta;

  uint16_t  pid_p;
  uint16_t  pid_i;
  uint16_t  pid_d;
  uint16_t  pid_divisor;

  uint8_t   joystick_xAxisDeadzone;

  uint8_t   cruiseSpeed_defaultSpeed;
  uint8_t   cruiseSpeed_manualMaxSpeed;

  uint16_t  offLineMaxTime;  

  uint16_t  info_printValsInterval;  
  bool      debugFlag;
  
} cb_linefollowing_config;



CB_NV_ID(1, NV_PID_P, "PID_P", cb_linefollowing_config, pid_p);
CB_NV_ID(2, NV_PID_I, "PID_I", cb_linefollowing_config, pid_i);
CB_NV_ID(3, NV_PID_D, "PID_D", cb_linefollowing_config, pid_d);
CB_NV_ID(4, NV_IRBIAS_1, "IR_BIAS_1", cb_linefollowing_config, ir_bias_1);
CB_NV_ID(5, NV_IRBIAS_2, "IR_BIAS_2", cb_linefollowing_config, ir_bias_2);
CB_NV_ID(6, NV_IRBIAS_3, "IR_BIAS_3", cb_linefollowing_config, ir_bias_3);


#endif

