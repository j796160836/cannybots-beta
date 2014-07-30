#ifndef _CANNYBOTSRACER_H
#define _CANNYBOTSRACER_H
#include <CannybotsTypes.h>

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

// constants for 'move'
#define CANNYBOTSRACER_MOVE_STOP     0
#define CANNYBOTSRACER_MOVE_GO       1
#define CANNYBOTSRACER_MOVE_LEFT     2
#define CANNYBOTSRACER_MOVE_RIGHT    3
#define CANNYBOTSRACER_MOVE_SHUFFLE  4

// constants for 'test'

#define CANNYBOTSRACER_TEST_MOTORS  1
#define CANNYBOTSRACER_TEST_IR_1    2


#endif

