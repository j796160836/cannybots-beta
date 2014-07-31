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
CB_ID(10, RACER_MOVE, "move");
CB_ID(11, RACER_TEST, "test");
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
#define NV_VERSION                      0
#define NV_PID_ALGO_TYPE                4
#define NV_DEFAULT_CRUISESPEED          5
#define NV_MAX_MANUAL_CRUISESPEED       6
#define NV_MAX_MOTOR_SPEED              7
#define NV_MOTOR_ALGO_TYPE              8
#define NV_M1_A_PIN                     9
#define NV_M1_B_PIN                    10
#define NV_M1_SENSE_PIN                11
#define NV_M2_A_PIN                    12
#define NV_M2_B_PIN                    13
#define NV_M2_SENSE_PIN                14
#define NV_MDRIVER_MODE_PIN            15
#define NV_BAT_PIN                     16
//   bits [0..7] =  [ HAS_MDRRIVEMODE, HAT_MOTOR_SENSE, HAS_BAT_SENSE, M1_POSITIVESPEED_IS_FWD, M2_POSITIVESPEED_IS_FWD, n/a, n/a, n/a],
#define NV_FEATURES_MASK1              17
#define NV_IR_MAX                      18 // UINT
#define NV_IR_WHITE_THRESHOLD          20 // UINT
#define NV_XAXIS_DEADZONE              22 // BYTE
#define NV_BOT_ID                      23 // UINT
#define NV_MAX_MOTOR_DELTA             24
#define NV_MAX_MOTOR_DELTA_DIV         25
#define NV_IRPIN_1                     30
#define NV_IRPIN_2                     31
#define NV_IRPIN_3                     32
#define NV_IRBIAS_1                    40 // INT8
#define NV_IRBIAS_2                    41 // INT8
#define NV_IRBIAS_3                    42 // INT8
#define NV_PID_P                       50 // INT16
#define NV_PID_I                       52 // INT16
#define NV_PID_D                       54 // INT16

#endif

