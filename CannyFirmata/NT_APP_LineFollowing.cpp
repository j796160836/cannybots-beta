
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Line Following
#include "NTProtocol.h"
#include "NTUtils.h"



// Config 


void lf_cfg_set_pid_p(int16_t p) {
   NT_nv_setInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_P_VAL, p);
}
void lf_cfg_set_pid_i(int16_t i) {
   NT_nv_setInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_I_VAL, i);
}
void lf_cfg_set_pid_d(int16_t d) {
     NT_nv_setInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_D_VAL, d);
}

void lf_cfg_set_led_col(int8_t col) {
     NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL, col);
}

void lf_cfg_set_led_bri(int8_t bri) {
     NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBBRI_VAL, bri);
}

int16_t lf_cfg_get_pid_p() {  
  return NT_nv_getInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_P_VAL);
}
int16_t lf_cfg_get_pid_i() {
  return NT_nv_getInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_I_VAL);
}
int16_t lf_cfg_get_pid_d() {
  return NT_nv_getInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_D_VAL);
}

int16_t lf_cfg_get_led_col() {
  return NT_nv_getInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL);
}

int16_t lf_cfg_get_led_brightness() {
  return NT_nv_getInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBBRI_VAL);
}


void NT_nv_configDefaults_LineFollowing() {
  for (int a = NT_NV_CFG_APP_LINEFOLLOWING_BASE; a<8; a++) {
    NT_nv_setByte(a,0);
  }
  NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL, LINEFOLLOW_RGB_COLOUR_RED);  
  NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBBRI_VAL, 255);  
}


// Actions



void lf_go() {
  INFO_PRINTLN("Go!");
}

void lf_stop() {
  INFO_PRINTLN("Stop!");
}

void lf_left() {
  INFO_PRINTLN("LEFT");
}

void lf_right() {
  INFO_PRINTLN("RIGHT");
}

void lf_switch() {
    INFO_PRINTLN("Switch");
}

void lf_speed(int16_t speed) {
    INFO_PRINT("LS:");
    INFO_PRINTLN(speed);
}


void lf_motor_speed(uint8_t motor, int16_t speed) {
    INFO_PRINT("M");
    INFO_PRINT(motor);
    INFO_PRINT("=");
    INFO_PRINTLN(speed);
}


#define F(x) x
uint8_t  linefollow_processCommand(uint8_t cmd, uint8_t id, int16_t p1) {
  //debug(F("linefollow:CMD=%d\n"), cmd);
  //debug(F("linefollow:ID =%d\n"), id);
  int status = NT_STATUS_OK;
  switch (cmd) {
    case NT_CMD_LINEFOLLOW_MOVE:
      //debug(F("linefollow:MOVE_TYPE=%d\n"), id);
      switch (id) {
        case LINEFOLLOW_GO:   lf_go(); break;
        case LINEFOLLOW_STOP: lf_stop(); break;
        case LINEFOLLOW_LEFT: lf_left(); break;
        case LINEFOLLOW_RIGHT:lf_right(); break;
        case LINEFOLLOW_SWITCH_NEXT_JUNCTION:lf_switch(); break;
        case LINEFOLLOW_SPEED:lf_speed(p1); break;
        default:
          DBG(F("linefollow:NT_ERROR_CMD_INVALID for NT_CMD_LINEFOLLOW_MOVE"));
          status = NT_ERROR_CMD_INVALID;
      }       
      break;    
    case NT_CMD_LINEFOLLOW_CONFIG_SET:
      switch (id) {
        case LINEFOLLOW_CFG_PID_P:lf_cfg_set_pid_p(p1); break;
        case LINEFOLLOW_CFG_PID_I:lf_cfg_set_pid_i(p1); break;
        case LINEFOLLOW_CFG_PID_D:lf_cfg_set_pid_d(p1); break;
        case LINEFOLLOW_CFG_RGB_COLOUR: lf_cfg_set_led_col(p1); break;
        case LINEFOLLOW_CFG_RGB_BRIGHTNESS: lf_cfg_set_led_bri(p1); break;
        
        default:
          DBG(F("linefollow:NT_ERROR_CMD_INVALID for NT_CMD_LINEFOLLOW_CONFIG_SET"));
          status = NT_ERROR_CMD_INVALID;
      }       
      break;
    case NT_CMD_LINEFOLLOW_CONFIG_GET:
      
      switch (id) {
        case LINEFOLLOW_CFG_DEVICE_ID:  
          NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_CONFIG_GET, LINEFOLLOW_CFG_DEVICE_ID, nv_cfg_get_deviceId()); 
          break;
        case LINEFOLLOW_CFG_PID_P:  
          NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_CONFIG_GET, LINEFOLLOW_CFG_PID_P, lf_cfg_get_pid_p()); 
          break;
        case LINEFOLLOW_CFG_PID_I:
          NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_CONFIG_GET, LINEFOLLOW_CFG_PID_I, lf_cfg_get_pid_i()); 
          break;
        case LINEFOLLOW_CFG_PID_D:
          NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_CONFIG_GET, LINEFOLLOW_CFG_PID_D, lf_cfg_get_pid_d()); 
          break;
        case LINEFOLLOW_CFG_RGB_COLOUR: 
          NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_CONFIG_GET, LINEFOLLOW_CFG_RGB_COLOUR, lf_cfg_get_led_col()); 
          break;
        case LINEFOLLOW_CFG_RGB_BRIGHTNESS: 
          NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_CONFIG_GET, LINEFOLLOW_CFG_RGB_BRIGHTNESS, lf_cfg_get_led_brightness()); 
          break;
        default:
          DBG(F("linefollow:NT_ERROR_CMD_INVALID for NT_CMD_LINEFOLLOW_CONFIG_SET"));
          status = NT_ERROR_CMD_INVALID;
      }       
      break;
    case NT_CMD_LINEFOLLOW_MOTOR_SPEED:
        lf_motor_speed(id,p1); 
        break;
    default:
      DBG(F("linefollow:NT_ERROR_CMD_INVALID!"));
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}


