//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Line Following


// MUST HAVE!!!

// TODO:  DONE!  0) uC to remote control (app) data flow 
// TODO:  1) line following mode - increase and decrease cruise speed from the app joystick. 
// TODO:  DONE 2) autonomous switch to manual mode when out of line (that is enabling steering, just like you had before).
// TODO:  DONE 4) manual/line following button switch.
// TODO:  DONE 5)uncomment lf_report_stopped


// nice to have:

// 1. connection status icon on app
// 2. connection mangement screen
// 3. cruise speed in EEPROM and iOS app (note : IR sensor bias in eeprom and in app has been done) 
// 4. current settings (e.g. cruise speed), not just EEPROM values, sent to app on connection


#include "CannybotsLineFollowing.h"

//#define DEBUG 1
//#define DEBUG_OUT Serial
//#define DEBUG_OUT_TARGET_UART_BAUD 9600
//#define NT_PLATFORM_AVR 1

#include <stdint.h> 
#include <Arduino.h>
#include <EEPROM.h>

// NT & Line following deps.
#include <SimpleFIFO.h>
#include <PID_v1.h>
#include <NTUtils.h>
#include <NTProtocol.h>
#include <NTMessaging.h>



void motor(int speedA, int speedB);
void printvalues ();

#define INFO_LED 13

// PIN Assignments
// IR sensor pins
#define IR1 A9
#define IR2 A8 
#define IR3 A6

// motor control pins

// motor control pins
const int pinA1 = 5; 
const int pinA2 = 6; 
const int pinB1 = 10;
const int pinB2 = 11;

//reading from IR sensors
int IR1_val = 0, IR2_val = 0, IR3_val = 0;
bool ir1on = 0, ir2on = 0, ir3on = 0;

int IRbias[3] = {0, 0, 0};

volatile int speedA = 0;
volatile int speedB = 0;
volatile int manualA = 0;
volatile int manualB = 0;

int baseCruiseSpeed = 100;
int cruiseSpeed=baseCruiseSpeed;

int printdelay = 1; //counter to slow print rate

boolean isHalted = false;


// read the IR sensors:
//set limit on reading. The reading can be very high and inaccurate on pitch black


void read_ir_sensors(){
  //IR1_val = analogRead(IR1);
  IR1_val = constrain(analogRead(IR1) - IRbias[0], 0, IR_MAX); //left looking from behind
  ir1on = IR1_val>=IR_MAX;
  
  //IR2_val = analogRead(IR2);
  IR2_val = constrain(analogRead(IR2) - IRbias[1], 0, IR_MAX); //centre
  ir2on   = IR2_val>=IR_MAX;
  
  //IR3_val = analogRead(IR3);
  IR3_val = constrain (analogRead(IR3) - IRbias[2], 0, IR_MAX); //right
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
  static bool reportSent = false;
  static bool startReportSent = false;
  static unsigned long reportSentLast = millis();
  static unsigned long startReportSentLast = millis();

  if (IR2_val < IR_MAX) {        
    isHalted = false;
    // only report that we are back on the line once per detection, also throttle reporting in case of sensor fluctutions 
    if (!startReportSent) {
      if  ((millis() - startReportSentLast) > 1000) {
         lf_report_started ();
        startReportSentLast = millis();
      }
    }
    startReportSent=true;   
    reportSent=false;
  }
  
  if (isHalted) {
      motor(manualA, manualB);
      return;
  }
  read_ir_sensors();

  // process IR readings
  error_last = error; //store previous error before new one is caluclated
  //set bounds for error
  error = constrain(IR1_val - IR3_val, -30, 30);
  
  // calculate proportional term
  P_error = error * Kp;

  // calculate differential term
  D_error = (error - error_last)*Kd;

  correction = P_error + D_error;

    
  //if sernsor 2 is on white, set spped to zero
  if (IR2_val >= IR_MAX) {
    // set by app
      if (manualA>0) {
        cruiseSpeed = baseCruiseSpeed + manualA/2;
      }
      
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
    // only report that we're off the line once per detection, also throttle reporting in case of sensor fluctutions 
    if (!reportSent) {
      if  ((millis() - reportSentLast) > 1000) {
        lf_report_stopped();
        halt_motors();
        reportSentLast = millis();
      }
    }
    reportSent=true;
    startReportSent=false;
    speedA = manualA;
    speedB = manualB;
  }

  motor(speedA, speedB);

  printvalues();
}


// motor controller function
void motor(int speedA, int speedB)
{
 if (speedA >= 0){
    if (speedA > 255)
    speedA = 255;
   analogWrite(pinA1, speedA);
   analogWrite(pinA2,0);
  }
  
  if (speedA < 0){
   if (speedA < -255)
    speedA = -255; 
    analogWrite(pinA1,0);
    analogWrite(pinA2,speedA);
  }
  
  if (speedB >= 0){
    if (speedB > 255)
    speedB = 255;
   analogWrite(pinB1, speedB);
   analogWrite(pinB2,0);
  }
  
  if (speedB < 0){
   if (speedB < -255)
    speedB = -255; 
    analogWrite(pinB1,0);
    analogWrite(pinB2,speedB);
  }
  

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

void test_motor() {
   digitalWrite(INFO_LED,HIGH);
   motor(0,128);
   delay(250);
   motor(128,0);
   delay(250);
   motor(0,-128);
   delay(250);
   motor(-128,0);
   delay(250);
   motor(0,0);
   digitalWrite(INFO_LED,LOW);
}

void info_blink(int count, int onPause, int offPause) {
  while (count--) {
     digitalWrite(INFO_LED,HIGH);
     delay(onPause);
     digitalWrite(INFO_LED,LOW);
     delay(offPause);
  }     
}

void lf_switch() {
   DBG("Switch");
    test_motor();

}

void lf_speed(int16_t speed) {
    Serial.print("LS=");
    Serial.println(speed,DEC);
    cruiseSpeed=speed;
}

void lf_motor_speed(uint8_t motorNum, int16_t speed) {
    static int yAxisValue = 0;
    static int xAxisValue = 0;
    
    Serial.print("M");
    Serial.print(motorNum, DEC);
    Serial.print("=");
    Serial.println(speed, DEC);
    switch (motorNum) {
      case 1: 
        manualB = speed;
        break;
      case 2: 
         manualA=speed;
        break;
      case 3: 
        //joy X axis vale
        xAxisValue=speed;
        break;
      case 4: 
         //joy y axis vale
         yAxisValue=speed;
        break;
      default:
        Serial.println("unknown motor");      
    }
    
    if ((motorNum ==3) || (motorNum==4) ) {
      //throttle (Y axis) and direction (X axis)
    int throttle = yAxisValue;
    int direction = xAxisValue;
    int leftMotor,leftMotorScaled = 0; //left Motor helper variables
    float leftMotorScale = 0;

    int rightMotor,rightMotorScaled = 0; //right Motor helper variables
    float rightMotorScale = 0;

    float maxMotorScale = 0; //holds the mixed output scaling factor


    //mix throttle and direction
    leftMotor = throttle + direction;
    rightMotor = throttle - direction;

    //print the initial mix results
    Serial.print("LIN:"); Serial.print( leftMotor, DEC);
    Serial.print(", RIN:"); Serial.print( rightMotor, DEC);

    //calculate the scale of the results in comparision base 8 bit PWM resolution
    leftMotorScale =  leftMotor / 255.0;
    leftMotorScale = abs(leftMotorScale);
    rightMotorScale =  rightMotor / 255.0;
    rightMotorScale = abs(rightMotorScale);

    Serial.print("| LSCALE:"); Serial.print( leftMotorScale, 2);
    Serial.print(", RSCALE:"); Serial.print( rightMotorScale, 2);

    //choose the max scale value if it is above 1
    maxMotorScale = max(leftMotorScale, rightMotorScale);
    maxMotorScale = max(1, maxMotorScale);

    //and apply it to the mixed values
    leftMotorScaled = constrain(leftMotor / maxMotorScale, -255, 255);
    rightMotorScaled = constrain(rightMotor / maxMotorScale, -255, 255);

    Serial.print("| LOUT:"); Serial.print( leftMotorScaled);
    Serial.print(", ROUT:"); Serial.print( rightMotorScaled);

    Serial.println(" |");
   manualA= leftMotorScaled;
   manualB= rightMotorScaled;
    }
}

void lf_rgb_colour(uint8_t v) {
    //DBG("%d", v);
}

void lf_rgb_brightness(uint8_t v) {
    //DBG("%d", v);
}


void lf_ir_bias(uint8_t ir, int8_t val) {
   IRbias[ir] = val;
}

// Client Feedbak

void lf_report_stopped() {
  NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_MOVE, LINEFOLLOW_STOP, 0); 
}

void lf_report_started() {
  NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_MOVE, LINEFOLLOW_GO, 0); 
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


// IR BIAS cfg

void lf_cfg_set_ir_bias(uint8_t ir, int8_t val) {
  int nvOffset = NT_NV_CFG_APP_LINEFOLLOWING_IR_BIAS_BASE + ir;
  DBG("1");
  if ( (nvOffset >= NT_NV_CFG_APP_LINEFOLLOWING_IR_BIAS_BASE ) && ( nvOffset < NT_NV_CFG_APP_LINEFOLLOWING_IR_BIAS_MAX)) {
    DBG("2= (ir, offset, val)");
    DBG(ir);
    DBG(nvOffset);
    DBG(val);
    lf_ir_bias(ir, val);
    NT_nv_setByte(nvOffset, val);
  }
}

int16_t lf_cfg_get_ir_bias(uint8_t ir) {
  int nvOffset = NT_NV_CFG_APP_LINEFOLLOWING_IR_BIAS_BASE + ir;
  DBG("3");
  DBG("2= (ir, offset, val)");
  DBG(ir);
  DBG(nvOffset);

  if ( (nvOffset >= NT_NV_CFG_APP_LINEFOLLOWING_IR_BIAS_BASE ) && ( nvOffset < NT_NV_CFG_APP_LINEFOLLOWING_IR_BIAS_MAX)) {
    int v = (int8_t)NT_nv_getByte(nvOffset);
    DBG(v);
    return v;
  } else {
    DBG("err");
    return -2;
  }
}



void NT_nv_configDefaults_LineFollowing() {
  for (int a = NT_NV_CFG_APP_LINEFOLLOWING_BASE; a<8; a++) {
    NT_nv_setByte(a,0);
  }
  NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL, LINEFOLLOW_RGB_COLOUR_RED);  
  NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBBRI_VAL, 255);  
}



uint8_t  linefollow_processCommand(uint8_t cmd, uint8_t id, int16_t p1) {
  //debug(F("linefollow:CMD=%d\n"), cmd);
  //debug(F("linefollow:ID =%d\n"), id);
  digitalWrite(INFO_LED,HIGH);

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
          DBG("linefollow:NT_ERROR_CMD_INVALID for NT_CMD_LINEFOLLOW_MOVE");
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
        case LINEFOLLOW_CFG_IR_BIAS_1: 
        case LINEFOLLOW_CFG_IR_BIAS_2: 
        case LINEFOLLOW_CFG_IR_BIAS_3: 
        case LINEFOLLOW_CFG_IR_BIAS_4: 
        case LINEFOLLOW_CFG_IR_BIAS_5: 
        case LINEFOLLOW_CFG_IR_BIAS_6: 
        case LINEFOLLOW_CFG_IR_BIAS_7: 
        case LINEFOLLOW_CFG_IR_BIAS_8: 
        case LINEFOLLOW_CFG_IR_BIAS_9: 
        case LINEFOLLOW_CFG_IR_BIAS_10: 
          lf_cfg_set_ir_bias(id-LINEFOLLOW_CFG_IR_BIAS_1, p1);
          break;
        
        default:
          DBG("linefollow:NT_ERROR_CMD_INVALID for NT_CMD_LINEFOLLOW_CONFIG_SET");
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
        case LINEFOLLOW_CFG_IR_BIAS_1: 
        case LINEFOLLOW_CFG_IR_BIAS_2: 
        case LINEFOLLOW_CFG_IR_BIAS_3: 
        case LINEFOLLOW_CFG_IR_BIAS_4: 
        case LINEFOLLOW_CFG_IR_BIAS_5: 
        case LINEFOLLOW_CFG_IR_BIAS_6: 
        case LINEFOLLOW_CFG_IR_BIAS_7: 
        case LINEFOLLOW_CFG_IR_BIAS_8: 
        case LINEFOLLOW_CFG_IR_BIAS_9: 
        case LINEFOLLOW_CFG_IR_BIAS_10: 
        
          NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_CONFIG_GET, id, lf_cfg_get_ir_bias(id-LINEFOLLOW_CFG_IR_BIAS_1)); 
          break;
        default:
          DBG("linefollow:NT_ERROR_CMD_INVALID for NT_CMD_LINEFOLLOW_CONFIG_SET");
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
 digitalWrite(INFO_LED,LOW);

  return status;
}




void setup() {
  pinMode(INFO_LED, OUTPUT);
  // C
  info_blink(1,500,100);
  info_blink(3,200,100);
  delay(200);
  // B
  info_blink(1,500,100);
  info_blink(1,200,100);
  info_blink(1,500,100);
  info_blink(1,200,100);
  

  Serial.begin(9600);
  Serial1.begin(9600);
#ifdef ARDUINO_AVR_LEONARDO  
// while (!Serial) {
//    ; // wait for serial port to connect. Needed for Leonardo only
//}
#endif  
  Serial.println("UC_START");
  
  // restore config values
  lf_pid_p(lf_cfg_get_pid_p());
  lf_pid_i(lf_cfg_get_pid_i());
  lf_pid_d(lf_cfg_get_pid_d());
  lf_rgb_colour(lf_cfg_get_led_col());
  lf_rgb_brightness(lf_cfg_get_led_brightness());
  int i;
  for (i = 0; i < IR_BIAS_NUM_SENSORS; i++);
    lf_ir_bias(i, lf_cfg_get_ir_bias(i));
  
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
  

  lf_pid_setup();

  //test_motor();
   // TODO:  NT to support callback to linefollow_processCommand() above.
}


void loop() {
  NT_UART_parseIntoQueue(Serial1, inboundMsgFIFO);
  
  NT_processOutboundMessageQueue();
  lf_loop();
  NT_processInboundMessageQueue();
}











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


