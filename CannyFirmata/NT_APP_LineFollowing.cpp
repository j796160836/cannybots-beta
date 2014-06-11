//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Line Following
#import <Arduino.h>
#include <PID_v1.h>

#include "NT_myconfig.h"
#include "NTUtils.h"
#include "NTProtocol.h"
#include "NT_App_LineFollowing.h"


#define MOTOR_MAX_SPEED 255

#define IR_MAX 100
#define WHITE_THRESHOLD 100

void motor(int speedA, int speedB);
void printvalues ();

// PIN Assignments
// IR sensor pins
#define IR0 0
#define IR1 1
#define IR2 2 
#define IR3 3
#define IR4 4

// motor control pins

#define enableA 5      // analog input to control speed
#define phaseA 4       //digital output to control direction
#define enableB 6
#define phaseB 7

//reading from IR sensors
int IR1_val = 0, IR2_val = 0, IR3_val = 0;
bool ir1on = 0, ir2on = 0, ir3on = 0;
int IR1_bias = 4;
int IR2_bias = 0;
int IR3_bias = -3;

int speedA = 0;
int speedB = 0;
int manualA = 0;
int manualB = 0;

int cruiseSpeed=50;

int printdelay = 1; //counter to slow print rate

boolean isHalted = false;




void lf_setup()
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(phaseA, OUTPUT);
  pinMode(phaseB, OUTPUT);
}

// read the IR sensors:
//set limit on reading. The reading can be very high and inaccurate on pitch black


void read_ir_sensors(){
  IR1_val = analogRead(IR1);
  IR1_val = constrain(analogRead(IR1) - IR1_bias, 0, IR_MAX); //left looking from behind
  ir1on = IR1_val>=IR_MAX;
  
  IR2_val = analogRead(IR2);
  IR2_val = constrain(analogRead(IR2) - IR2_bias, 0, IR_MAX); //centre
  ir2on   = IR2_val>=IR_MAX;
  
  IR3_val = analogRead(IR3);
  IR3_val = constrain (analogRead(IR3) - IR3_bias, 0, IR_MAX); //right
  ir3on   = IR3_val>=IR_MAX;
  
  
}


#define PID_METHOD1

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef PID_METHOD1
int Kp = 0, Ki = 0, Kd = 0;

int I_limit = 100;

int P_error = 0;
int I_error = 0;
int D_error = 0;
int error = 0;
int error_last = 0; // to calculate D_error = error - error_last
int correction = 0; //error after PID filter


void lf_pid_setup(){
}

void lf_loop()
{
  // read the IR sensors:
  IR1_val = analogRead(IR1);
  IR1_val = analogRead(IR1) - IR1_bias; //left looking from behind
  //set limit on reading. The reading can be very high and inaccurate on pitch black
  if (IR1_val > 100)
  IR1_val = 100;
  
  IR2_val = analogRead(IR2);
  IR2_val = analogRead(IR2) - IR2_bias; //centre
  if (IR2_val > 100)
  IR2_val = 100;
  
  IR3_val = analogRead(IR3);
  IR3_val = analogRead(IR3) - IR3_bias; //right
  if (IR3_val > 100)
  IR3_val = 100;

  // process IR readings
  error_last = error; //store previous error before new one is caluclated
  //set bounds for error
  error = constrain(IR1_val - IR3_val, -30, 30);
  
  // calculate proportional term
  P_error = error * Kp;

  // calculate differential term
  D_error = (error - error_last)*Kd;

  correction = P_error + D_error;

  static bool reportSent = false;
    
  //if sernsor 2 is on white, set spped to zero
  if (IR2_val >= IR_MAX) {
    // set by app
    //cruiseSpeed = 100;
      // Set motor speed
      // If correction is > 0, increase the speed of motor A and 
      //decrease speed of motor B. If correction is < 0,  
      // decrease speed of A and increase speedB.
      speedA = constrain(cruiseSpeed + correction, -150, 150);
      speedB = constrain(cruiseSpeed - correction, -150, 150);
     reportSent = false;
  }
  else
  {
    if (!reportSent) {
      lf_report_stopped();
    }
    reportSent=true;
    speedA = manualA;
    speedB = manualB;
  }

  motor(speedA, speedB);

  printvalues();
}


// motor controller function
void motor(int _speedA, int _speedB)
{
  _speedA=constrain(_speedA, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  _speedB=constrain(_speedB, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  
  digitalWrite(phaseA, ( _speedA >= 0 ) ? HIGH:LOW);
  digitalWrite(phaseB, ( _speedB >= 0 ) ? HIGH:LOW);
  
  // map from the max range to a smaller subset
  _speedA=MOTOR_MAX_SPEED-abs(_speedA);
  _speedB=MOTOR_MAX_SPEED-abs(_speedB);
  //Serial.print("A=");  Serial.print(_speedA);  Serial.print(", B=");  Serial.println(_speedB);
  analogWrite(enableA, _speedA);
  analogWrite(enableB, _speedB);

}

void halt_motors() {
   isHalted = true;
   manualB = manualA = 0;
   motor(0,0);
}
void resume_motors() {
   isHalted = false;
   manualB = manualA = 0;
   motor(0,0);
}



void lf_pid_p(int16_t v) {
    Kp=v;
}
void lf_pid_i(int16_t v) {
    Ki=v;
}
void lf_pid_d(int16_t v) {
    Kd=v;
}


void printvalues ()
{
  if ( printdelay++ % 100 )
    return;
  Serial.print(IR1_val);
  Serial.print(", ");
  Serial.print(IR2_val);
  Serial.print(", ");
  Serial.print(IR3_val);
  Serial.print(", Kp=");
  Serial.print(Kp);
  Serial.print(", Kd=");
  Serial.print(Kd);
  Serial.print(", e=");
  Serial.print(error);
  Serial.print(", pe");
  Serial.print(P_error);
  Serial.print(", de=");
  Serial.print(D_error);
  Serial.print(", A=");
  Serial.print(speedA);
  Serial.print(", B=");
  Serial.println(speedB);
}

#endif


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef PID_METHOD2
double Setpoint, Input, Output;
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



void lf_pid_setup(){
   read_ir_sensors();
   Setpoint = 0;
   myPID.SetSampleTime(1);
   myPID.SetOutputLimits(-MOTOR_MAX_SPEED+105, MOTOR_MAX_SPEED-105);
   myPID.SetMode(AUTOMATIC);
}

void lf_loop()
{
    if (isHalted) {
        motor(manualA, manualB);
      return;
    }
   read_ir_sensors();
   printvalues(); 
  /*
  // check for manual mode
  if (IR2_val < WHITE_THRESHOLD) {
      // only flag stopped as a one-shot
     if (myPID.GetMode() != MANUAL) {
       lf_report_stopped();
       // pause PID
       myPID.SetMode(MANUAL);
     }
     
     speedA = manualA;
     speedB = manualB;
  } else {
    // one-shot mode change
     if (myPID.GetMode() != AUTOMATIC) {
        myPID.SetMode(AUTOMATIC);
     }

     myPID.Compute(); 
     speedA = cruiseSpeedManual + Output;
     speedB = cruiseSpeedManual - Output;
  }
  */
  Input =  2 - ( ( !ir1on + 2*(ir2on) + 3*!ir3on )  / ( !ir1on + ir2on + !ir3on) );
  myPID.Compute(); 
  speedA = cruiseSpeed + Output;
  speedB = cruiseSpeed - Output;
  motor(speedA, speedB);
}



// motor controller function
void motor(int _speedA, int _speedB)
{
  _speedA=constrain(_speedA, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  _speedB=constrain(_speedB, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  
  digitalWrite(phaseA, ( _speedA >= 0 ) ? HIGH:LOW);
  digitalWrite(phaseB, ( _speedB >= 0 ) ? HIGH:LOW);
  
  // map from the max range to a smaller subset
  _speedA=MOTOR_MAX_SPEED-abs(_speedA);
  _speedB=MOTOR_MAX_SPEED-abs(_speedB);
  //_speedA = map(_speedA, 255, 0, 100, 0);
  //_speedB = map(_speedB, 255, 0, 100, 0);
  //Serial.print("A=");  Serial.print(_speedA);  Serial.print(", B=");  Serial.println(_speedB);
  analogWrite(enableA, _speedA);
  analogWrite(enableB, _speedB);

}

void halt_motors() {
  isHalted = true;
   manualB = manualA = 0;
   myPID.SetMode(MANUAL);
   motor(0,0);
}
void resume_motors() {
   isHalted = false;
   manualB = manualA = 0;
   myPID.SetMode(MANUAL);
   motor(0,0);
}



void lf_pid_p(int16_t v) {
    //DBG("%d", v);
    Kp=((double)v)/100.0;
    myPID.SetTunings(Kp, Ki, Kd);

}
void lf_pid_i(int16_t v) {
    //DBG("%d", v);
    Ki=((double)v)/100.0;
    myPID.SetTunings(Kp, Ki, Kd);

}
void lf_pid_d(int16_t v) {
    //DBG("%d", v);
    Kd=((double)v)/100.0;
    myPID.SetTunings(Kp, Ki, Kd);

}


void printvalues ()
{
  if ( printdelay++ % 100 )
    return;
    
  Serial.print(IR1_val);
  Serial.print(", ");
  Serial.print(IR2_val);
  Serial.print(", ");
  Serial.print(IR3_val);
  Serial.print(", Kp=");
  Serial.print(Kp);
  Serial.print(", Kd=");
  Serial.print(Kd);
  Serial.print(", Input=");
  Serial.print(Input);
  Serial.print(", Output=");
  Serial.print(Output);
  Serial.print(", cruiseSpeed=");
  Serial.print(cruiseSpeed);
  
  Serial.print(", A=");
  Serial.print(speedA);
  Serial.print(", B=");
  Serial.println(speedB);
  
}

#endif

//////////////////////////////////////////
// Actions

void lf_go() {
  DBG("Go!");
  resume_motors();
}

void lf_stop() {
  DBG("Stop!");
  halt_motors();
}

void lf_left() {
  DBG("LEFT");
}

void lf_right() {
  DBG("RIGHT");
}

void lf_switch() {
    DBG("Switch");
}

void lf_speed(int16_t speed) {
    //DBG("LS=%d", speed);
    cruiseSpeed=speed;
}


void lf_motor_speed(uint8_t motorNum, int16_t speed) {
    //DBG("M, %d=%d", motor, speed);
    if (2==motorNum) {
        manualA=speed;
    }
     else {
        manualB=speed;
     }
}

void lf_rgb_colour(uint8_t v) {
    //DBG("%d", v);
}

void lf_rgb_brightness(uint8_t v) {
    //DBG("%d", v);
}


// Client Feedbak

void lf_report_stopped() {
  NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_MOVE, LINEFOLLOW_STOP, 0); 
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
   pinMode(enableA, OUTPUT);
   pinMode(enableB, OUTPUT);
   
   lf_pid_setup();
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


