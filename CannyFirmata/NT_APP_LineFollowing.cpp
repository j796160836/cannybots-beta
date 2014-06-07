//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Line Following
#import <Arduino.h>

#include "NT_myconfig.h"
#include "NTUtils.h"
#include "NTProtocol.h"
#include "NT_App_LineFollowing.h"

void motor(int speedA, int speedB);
void printvalues ();


// Constants


// PIN Assignments
// IR sensor pins
const int IR1 = 0;  // Analog input pin that the potentiometer is attached to
const int IR2 = 1; // Analog output pin that the LED is attached to
const int IR3 = 2;
// motor control pins
const int enablePinA = 5;
const int enablePinB = 6;


// variable definitions
int IR1_val = 0; //reading from IR sensor 1
int IR2_val = 0;
int IR3_val = 0;
int mean_val = 0;
int error = 0;
//int white_mean = 22;
//int black_mean = 33;
int IR1_bias = -1;
int IR2_bias = 0;
int IR3_bias = 1;

int speedA = 0;
int speedB = 0;

// Gain setting
int speedScaling = 0;
int cruiseSpeed = 0;

int Kp = 0;
int Ki = 0;
int Kd = 0;
int I_limit = 100;

int P_error = 0;
int I_error = 0;
int D_error = 0;
int error_last = 0; // to calculate D_error = error - error_last
//int correction = 0; //error after PID filter
int correctionA = 0; //error after PID filter
int correctionB = 0; //error after PID filter
int printdelay = 1; //counter to slow print rate




//////////////////////////////////////////
// Init


void lf_loop() {

  // read the IR sensors:
  //delay(25);
  analogRead(IR1);
  IR1_val = analogRead(IR1) - IR1_bias; //left looking from behind
  
  //delay(50);
  analogRead(IR2);
  IR2_val = analogRead(IR2) - IR2_bias; //centre
  
  //delay(50);
  analogRead(IR3);
  IR3_val = analogRead(IR3) - IR3_bias; //right

  // process IR readings
  mean_val = (IR1_val + IR2_val + IR3_val) / 3;
  error_last = error; //store previous error before new one is caluclated
  error = IR1_val - IR3_val;

  //Kp = 8;
  //Ki = 0;
  //Kd = 3;

  // calculate proportional term
  P_error = error * Kp;

  // calculate integral term
  I_error = I_error + error;
  if (I_error > I_limit)
    I_error = I_limit;
  if (I_error < (-I_limit))
    I_error = -I_limit;

  // calculate differential term
  D_error = (error_last - error)*Kd;

  //correction = P_error + D_error;

  speedA = (cruiseSpeed + correctionA);
  speedB = (cruiseSpeed + correctionB);

  // set motor speed
  /*
  if (correction < 0)
    speedA = (cruiseSpeed + correction);
  else
    speedA = cruiseSpeed;

  if (correction > 0)
    speedB = (cruiseSpeed - correction);
  else
    speedB = cruiseSpeed;
  */
  motor(speedA, speedB);


  // print values
  if (printdelay == 20) {
    //printvalues();
    printdelay = 1;
  }
  else {
    printdelay = printdelay + 1;
  }
}

// motor controller function
void motor(int speedA, int speedB)
{

  // deadband
  if (speedA < 5)
    speedA = 0;
  if (speedB < 5)
    speedB = 0;
  // upper cutoff
  if (speedA > 255)
    speedA = 255;
  if (speedB > 255)
    speedB = 255;

  analogWrite(enablePinA, (255 - speedA));
  analogWrite(enablePinB, (255 - speedB));

}

void printvalues ()
{

  Serial.print(IR1_val);
  Serial.print(", ");
  Serial.print(IR2_val);
  Serial.print(", ");
  Serial.print(IR3_val);
  Serial.print(", ");
  Serial.print(error);
  Serial.print(", ");
  Serial.print(P_error);
  Serial.print(", ");
  Serial.print(D_error);
  Serial.print(", ");
  Serial.print(speedA);
  Serial.print(", ");
  Serial.println(speedB);
}


//////////////////////////////////////////
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
    INFO_PRINT("LS=");
    INFO_PRINTLN(speed);
    cruiseSpeed=speed;
}


void lf_motor_speed(uint8_t motor, int16_t speed) {
    INFO_PRINT("M");
    INFO_PRINT(motor);
    INFO_PRINT("=");
    INFO_PRINTLN(speed);
    if (2==motor) {
      //speedA=speed;
      if (speed>=0)
        correctionA=speed/4;
    }
     else {
      if (speed>=0)
        correctionB=speed/4;
     }
}

void lf_pid_p(int16_t v) {
    INFO_PRINT("P=");
    INFO_PRINTLN(v);
    Kp=v;
}
void lf_pid_i(int16_t v) {
    INFO_PRINT("I=");
    INFO_PRINTLN(v);
    Ki=v;
}
void lf_pid_d(int16_t v) {
    INFO_PRINT("D=");
    INFO_PRINTLN(v);
    Kd=v;
}
void lf_rgb_colour(uint8_t v) {
    INFO_PRINT("L=");
    INFO_PRINTLN(v);
}

void lf_rgb_brightness(uint8_t v) {
    INFO_PRINT("B=");
    INFO_PRINTLN(v);
}


//////////////////////////////////////////
// Config 


void lf_cfg_set_pid_p(int16_t p) {
  lf_pid_p(p);
   NT_nv_setInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_P_VAL, p);
}
void lf_cfg_set_pid_i(int16_t i) {
  lf_pid_i(i);
   NT_nv_setInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_I_VAL, i);
}
void lf_cfg_set_pid_d(int16_t d) {
  lf_pid_d(d);
     NT_nv_setInt(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_D_VAL, d);
}

void lf_cfg_set_led_col(int8_t col) {
  lf_rgb_colour(col);
     NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL, col);
}

void lf_cfg_set_led_bri(int8_t bri) {
  lf_rgb_brightness(bri);
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
  return NT_nv_getByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL);
}

int16_t lf_cfg_get_led_brightness() {
  return NT_nv_getByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBBRI_VAL);
}


void NT_nv_configDefaults_LineFollowing() {
  for (int a = NT_NV_CFG_APP_LINEFOLLOWING_BASE; a<8; a++) {
    NT_nv_setByte(a,0);
  }
  NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL, LINEFOLLOW_RGB_COLOUR_RED);  
  NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBBRI_VAL, 255);  
}

void lf_init() {
  // restore config values
  lf_pid_p(lf_cfg_get_pid_p());
  lf_pid_i(lf_cfg_get_pid_i());
  lf_pid_d(lf_cfg_get_pid_d());
  lf_rgb_colour(lf_cfg_get_led_col());
  lf_cfg_set_led_bri(lf_cfg_get_led_brightness());
  
  // TODO:  Setup motor driver
   pinMode(enablePinA, OUTPUT);
   pinMode(enablePinB, OUTPUT);
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
        case LINEFOLLOW_CFG_DEVICE_ID: nv_cfg_set_deviceId(p1); break;
        
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


