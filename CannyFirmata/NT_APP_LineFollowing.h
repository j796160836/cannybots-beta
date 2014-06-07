#ifndef NT_APP_LineFollowing_h
#define NT_APP_LineFollowing_h

#include "NTProtocol.h"
#include "NTUtils.h"
#include <stdint.h> 

// EEPROM settings 

#define NT_NV_CFG_APP_LINEFOLLOWING_BASE  32

// Understand that chaging these offsets will BREAK config for existing bots in the field
#define NT_NV_CFG_APP_LINEFOLLOWING_P_VAL 0
#define NT_NV_CFG_APP_LINEFOLLOWING_I_VAL 2
#define NT_NV_CFG_APP_LINEFOLLOWING_D_VAL 4
#define NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL 8
#define NT_NV_CFG_APP_LINEFOLLOWING_RGBBRI_VAL 9




// Command processing

#define NT_CAT_APP_LINEFOLLOW NT_CAT_APP

// Commands:

#define NT_CMD_LINEFOLLOW_MOVE 1
#define NT_CMD_LINEFOLLOW_CONFIG_SET 2
#define NT_CMD_LINEFOLLOW_CONFIG_GET 3
#define NT_CMD_LINEFOLLOW_MOTOR_SPEED 4



// id's for NT_CMD_LINEFOLLOW_MOVE
#define LINEFOLLOW_STOP 1
#define LINEFOLLOW_GO 2
#define LINEFOLLOW_LEFT 3
#define LINEFOLLOW_RIGHT 4
#define LINEFOLLOW_SWITCH_NEXT_JUNCTION 5
#define LINEFOLLOW_SPEED 6



// id's for NT_CMD_LINEFOLLOW_CONFIG_SET / NT_CMD_LINEFOLLOW_CONFIG_GET
#define LINEFOLLOW_CFG_DEVICE_ID  0

#define LINEFOLLOW_CFG_PID_P  1
#define LINEFOLLOW_CFG_PID_I  2
#define LINEFOLLOW_CFG_PID_D  3

// chose one of the LINEFOLLOW_RGB_COLOUR_[xxx] below
#define LINEFOLLOW_CFG_RGB_COLOUR  4

// Range 0-255
#define LINEFOLLOW_CFG_RGB_BRIGHTNESS  5


// Values for LINEFOLLOW_RGB_COLOUR 
#define LINEFOLLOW_RGB_COLOUR_OFF  0
#define LINEFOLLOW_RGB_COLOUR_RED  1
#define LINEFOLLOW_RGB_COLOUR_GREEN  2
#define LINEFOLLOW_RGB_COLOUR_BLUE  3
#define LINEFOLLOW_RGB_COLOUR_WHITE  4



void lf_init();
void lf_loop();
uint8_t  linefollow_processCommand(uint8_t cmd, uint8_t id, int16_t p1);


#endif
