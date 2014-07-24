  //////////////////////////////////////////////////////////////////////////////////////////////////
//
// Line Following

// TODO:
// 1. connection status icon on app
// 2. connection mangement screen
// 3. cruise speed in EEPROM and iOS app (note : IR sensor bias in eeprom and in app has been done)
// 4. current settings (e.g. cruise speed), not just EEPROM values, sent to app on connection


#include <stdint.h>
#include <Arduino.h>
#include <EEPROM.h>

// NT & Line following deps.
#include <SimpleFIFO.h>
#include <PID_v1.h>
#include <NTUtils.h>
#include <NTProtocol.h>
#include <NTMessaging.h>

#include "CannybotsLineFollowing.h"
void lf_report_followingMode(bool isLineMode);



#ifdef USE_ANALOG_LIB
// see: https://github.com/merose/AnalogScanner
#include <AnalogScanner.h>
AnalogScanner scanner;
#define ANALOG_READ scanner.getValue
#else
#define ANALOG_READ analogRead
#endif

#define V7

#ifdef V7
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
#else 
#define IR1 A8
#define IR2 A9
#define IR3 A10

// motor control pins

// motor control pins
const int pinA1 = 6; 
const int pinA2 = 5; 
const int pinB1 = 4;
const int pinB2 = 3;
const int pin_MODE = 7;

#endif

//reading from IR sensors
int IR1_val = 0, IR2_val = 0, IR3_val = 0;
bool ir1on = 0, ir2on = 0, ir3on = 0;

int IRbias[IR_BIAS_NUM_SENSORS] = {0, 0, 0};

int speedA = 0;
int speedB = 0;
int manualA = 0;
int manualB = 0;

int yAxisValue = 0;
int xAxisValue = 0;


int baseCruiseSpeed = 80;
int cruiseSpeed = baseCruiseSpeed;

boolean isLineFollowingMode = true;

volatile unsigned long lastCommandTime = millis();



// read the IR sensors:
// set limit on reading. The reading can be very high and inaccurate on pitch black
void read_ir_sensors() {

  IR1_val = constrain(ANALOG_READ(IR1) - IRbias[0], 0, IR_MAX); //left looking from behind
  IR2_val = constrain(ANALOG_READ(IR2) - IRbias[1], 0, IR_MAX); //centre
  IR3_val = constrain(ANALOG_READ(IR3) - IRbias[2], 0, IR_MAX); //right

  ir1on = IR1_val >= IR_MAX;
  ir2on = IR2_val >= IR_MAX;
  ir3on = IR3_val >= IR_MAX;
}


void motor_V5(int _speedA, int _speedB)
{
   _speedA = constrain(_speedA, -255, 255);
   _speedB = constrain(_speedB, -255, 255);
   
   digitalWrite(pinA1,_speedA >=0 ? HIGH:LOW) ;
   analogWrite (pinA2, abs(_speedA));
   
   digitalWrite(pinB1, _speedB >=0 ? LOW:HIGH);
   analogWrite (pinB2,abs(_speedB));
}


// motor controller function
void motor(int _speedA, int _speedB) // V4
{
  _speedA = constrain(_speedA, -255, 255);
  _speedB = constrain(_speedB, -255, 255);
  if (_speedA >= 0) {
    analogWrite(pinA1, _speedA);
    analogWrite(pinA2, 0);
  } else {
    analogWrite(pinA1, 0);
    analogWrite(pinA2, abs(_speedA));
  }
  if (_speedB >= 0) {
    analogWrite(pinB1, _speedB);
    analogWrite(pinB2, 0);
  } else {
    analogWrite(pinB1, 0);
    analogWrite(pinB2, abs(_speedB));
  }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PID_METHOD1

#ifdef PID_METHOD1
int Kp = 0, Ki = 0, Kd = 0;
int I_limit = 100;
int P_error = 0;
int I_error = 0;
int D_error = 0;
int error = 0;
int error_last = 0; // to calculate D_error = error - error_last
int correction = 0; //error after PID filter



void printvalues ()
{
  static unsigned long lastPrint = millis();
  if ( millis() - lastPrint < 250 )  return;
  lastPrint = millis();
  
  DBG_PRINTF(    "%lu: IR(%u,%u,%u) Kpd(%d,%d) e(%d) PeDe(%d,%d) Sab(%d,%d) Mab(%d,%d), XY(%d,%d)",
                 millis(),
                 IR1_val, IR2_val, IR3_val,
                 Kp, Kd, error, P_error, D_error,
                 speedA, speedB, manualA, manualB,
                 xAxisValue, yAxisValue
            );
}


void lf_pid_setup() {

}

void lf_loop()
{
  read_ir_sensors();

  isLineFollowingMode =  IR2_val >= WHITE_THRESHOLD;
  lf_report_followingMode(isLineFollowingMode);


  if (isLineFollowingMode) {
    // process IR readings via PID
    error_last = error;                                   // store previous error before new one is caluclated
    error = constrain(IR1_val - IR3_val, -30, 30);        // set bounds for error
    P_error = error * Kp/10;                                 // calculate proportional term
    D_error = (error - error_last) * Kd/10;                  // calculate differential term
    correction = P_error + D_error;
    cruiseSpeed = baseCruiseSpeed + manualA;
    speedA = constrain(cruiseSpeed - correction, -200, 200);
    speedB = constrain(cruiseSpeed + correction, -200, 200);
  } else {
    // in manual mode
    if ((millis() - lastCommandTime) > 2000) {
      // no command has been received in the last 2 seconds, err on the side of caution and stop!
      speedA = speedB = 0;
    } else {
      // 
      speedA= manualA;
      speedB= manualB;
    }
  }
  printvalues();
  motor(speedA, speedB);
}


void halt_motors() {
  isLineFollowingMode = false;
}
void resume_motors() {
  isLineFollowingMode = true;
}



void lf_pid_p(int16_t v) {
  Kp = v;
}
void lf_pid_i(int16_t v) {
  Ki = v;
}
void lf_pid_d(int16_t v) {
  Kd = v;
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
  static unsigned long lastTime = millis();
  static byte ledOn = 0;
  if (millis() - lastTime > 1000) {
    lastTime = millis();
    digitalWrite(INFO_LED, ledOn ? HIGH : LOW);
    ledOn = 1 - ledOn;
  }

}

void info_blink(int count, int onPause, int offPause) {
  while (count--) {
    digitalWrite(INFO_LED, HIGH);
    delay(onPause);
    digitalWrite(INFO_LED, LOW);
    delay(offPause);
  }
}

void lf_switch() {
  DBG("Switch");
}

void lf_speed(int16_t speed) {
  Serial.print("LS=");
  Serial.println(speed, DEC);
  manualA = speed;
}



void calculateMotorSpeedFromJoystick(int xAxisValue, int yAxisValue, int* motor1, int* motor2) {
  // direction (X axis)  
  // throttle  (Y axis)
  
  // handle throttle
  #define MOTOR_MAX_SPEED 255
  bool isForward = yAxisValue > 0;  
  long x2 = (long)xAxisValue*(long)xAxisValue;
  long y2 = (long)yAxisValue*(long)yAxisValue;
  
  int throttle = constrain(sqrt( x2 + y2) , -MOTOR_MAX_SPEED,MOTOR_MAX_SPEED);
  throttle = map (throttle, -255, 255, 270, 270 + 180);
  throttle = sin( radians(throttle) ) * MOTOR_MAX_SPEED;

  float throttleRatio = (1.0/MOTOR_MAX_SPEED)* throttle;

  

  int leftMotorSpeed  = 0;
  int rightMotorSpeed = 0;
  rightMotorSpeed = leftMotorSpeed  = throttleRatio * MOTOR_MAX_SPEED; 


  // handle direction  
  // only calc left & right if outside of X axis deadzone
  unsigned int xMag  = abs(xAxisValue);

  if ( xMag > 25  ) {
    bool isLeft = xAxisValue<0;
    int direction = map (xAxisValue, -255, 255, 270, 270 + 180);
    direction = sin( radians(direction) ) * MOTOR_MAX_SPEED;

    float directionRatio = abs((1.0/MOTOR_MAX_SPEED) * direction);
    
    int speed = throttleRatio * MOTOR_MAX_SPEED;
    if (isLeft) {
        leftMotorSpeed  = (1.0-directionRatio*2) * speed;
        rightMotorSpeed = directionRatio * speed;
    } else {
        leftMotorSpeed  = directionRatio * speed;      
        rightMotorSpeed = (1.0-directionRatio*2) * speed;
    }    
  } 
  
  // re-apply fwd/back sign.
  leftMotorSpeed = leftMotorSpeed * (isForward?1:-1);
  rightMotorSpeed = rightMotorSpeed * (isForward?1:-1);

  
  DBG_PRINTF("%lu: xAxisValue,yAxisValue(%d,%d) =  throttle(%d),  Left,Right(%d,%d)", millis(), xAxisValue, yAxisValue, throttle, leftMotorSpeed, rightMotorSpeed);

  *motor1 = rightMotorSpeed;
  *motor2 = leftMotorSpeed;
}


void lf_motor_speed(uint8_t motorNum, int16_t speed) {

  switch (motorNum) {
    case 1:
      manualB = speed;
      break;
    case 2:
      manualA = speed;
      break;
    case 3:
      //joy X axis vale  = Direction  -255 to 255
      xAxisValue = speed;
      break;
    case 4:
      //joy y axis vale = Throttle    -255 to 255
      yAxisValue = speed;
      break;
    default:
      Serial.println("unknown motor");
  }
  if ((motorNum == 3) || (motorNum == 4) ) {
    if (isLineFollowingMode) {
      manualA = yAxisValue > 0 ? yAxisValue : 0;
    } else if ( (0 == xAxisValue) && (yAxisValue == 0) ) {
      manualA = manualB = 0;
      motor(0, 0);                  // immediate stop
    } else {
      calculateMotorSpeedFromJoystick(xAxisValue, yAxisValue, &manualA, &manualB);
    }
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

void lf_report_followingMode(bool isLineMode) {
  static unsigned long lastCall = millis();
  // throttle sending to 1000/x times a second
  if (millis() - lastCall > 500) {
    lastCall = millis();
    if (isLineMode) {
      NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_MOVE, LINEFOLLOW_GO, 0);
    } else {
      NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_MOVE, LINEFOLLOW_STOP, 0);
    }
  }
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
  for (int a = NT_NV_CFG_APP_LINEFOLLOWING_BASE; a < 8; a++) {
    NT_nv_setByte(a, 0);
  }
  for (int b = NT_NV_CFG_APP_LINEFOLLOWING_IR_BIAS_BASE; b < NT_NV_CFG_APP_LINEFOLLOWING_IR_BIAS_MAX; b++) {
    NT_nv_setByte(b, 0);
  }
  NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBCOL_VAL, LINEFOLLOW_RGB_COLOUR_RED);
  NT_nv_setByte(NT_NV_CFG_APP_LINEFOLLOWING_BASE + NT_NV_CFG_APP_LINEFOLLOWING_RGBBRI_VAL, 255);
}



uint8_t  linefollow_processCommand(uint8_t cmd, uint8_t id, int16_t p1) {
  //debug(F("linefollow:CMD=%d\n"), cmd);
  //debug(F("linefollow:ID =%d\n"), id);
  //digitalWrite(INFO_LED, HIGH);
  lastCommandTime = millis();



  int status = NT_STATUS_OK;
  switch (cmd) {
    case NT_CMD_LINEFOLLOW_MOVE:
      //debug(F("linefollow:MOVE_TYPE=%d\n"), id);
      switch (id) {
        case LINEFOLLOW_GO:   lf_go(); break;
        case LINEFOLLOW_STOP: lf_stop(); break;
        case LINEFOLLOW_LEFT: lf_left(); break;
        case LINEFOLLOW_RIGHT: lf_right(); break;
        case LINEFOLLOW_SWITCH_NEXT_JUNCTION: lf_switch(); break;
        case LINEFOLLOW_SPEED: lf_speed(p1); break;
        default:
          DBG("linefollow:NT_ERROR_CMD_INVALID for NT_CMD_LINEFOLLOW_MOVE");
          status = NT_ERROR_CMD_INVALID;
      }
      break;
    case NT_CMD_LINEFOLLOW_CONFIG_SET:
      switch (id) {
        case LINEFOLLOW_CFG_PID_P: lf_cfg_set_pid_p(p1); break;
        case LINEFOLLOW_CFG_PID_I: lf_cfg_set_pid_i(p1); break;
        case LINEFOLLOW_CFG_PID_D: lf_cfg_set_pid_d(p1); break;
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
          lf_cfg_set_ir_bias(id - LINEFOLLOW_CFG_IR_BIAS_1, p1);
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

          NT_sendCommand(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_CONFIG_GET, id, lf_cfg_get_ir_bias(id - LINEFOLLOW_CFG_IR_BIAS_1));
          break;
        default:
          DBG("linefollow:NT_ERROR_CMD_INVALID for NT_CMD_LINEFOLLOW_CONFIG_GET");
          status = NT_ERROR_CMD_INVALID;
      }
      break;
    case NT_CMD_LINEFOLLOW_MOTOR_SPEED:
      lf_motor_speed(id, p1);
      break;
    default:
      DBG(F("linefollow:NT_ERROR_CMD_INVALID!"));
      status = NT_ERROR_CMD_INVALID;
  }
  //digitalWrite(INFO_LED, LOW);

  return status;
}


void setup() {
#ifdef IGNORE && ARDUINO_AVR_A_STAR_32U4
  // brownout detection
  // from:  http://www.pololu.com/docs/0J61/7
  pinMode(13, OUTPUT);
  if (MCUSR & (1 << BORF))
  {
    // A brownout reset occurred.  Blink the LED
    // quickly for 2 seconds.
    for (uint8_t i = 0; i < 2; i++)
    {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
  }
  MCUSR = 0; 
#else
//#error not comiling using the Polulu A* bootload, are u sure you dont want brownout detection? if so comment this line
#endif

#ifdef USE_ANALOG_LIB
  int scanOrder[IR_BIAS_NUM_SENSORS] = {IR1, IR2, IR3};
  scanner.setScanOrder(IR_BIAS_NUM_SENSORS, scanOrder);
  scanner.beginScanning();
  delay(1); // Wait for the first scans to occur.
#endif

#if 0
  pinMode(INFO_LED, OUTPUT);
  // C
  info_blink(1, 500, 100);
  info_blink(3, 200, 100);
  delay(200);
  // B
  info_blink(1, 500, 100);
  info_blink(1, 200, 100);
  info_blink(1, 500, 100);
  info_blink(1, 200, 100);
#endif

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
#ifdef V8
  pinMode(pin_MODE, OUTPUT);
  digitalWrite(pin_MODE, HIGH); //to set controller to Phase/Enable mode
#endif
  digitalWrite(13, HIGH);

}



void loop() {
  NT_UART_parseIntoCallback(Serial1, NT_message_enqueueInboundMessage);
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



void lf_pid_setup() {
  read_ir_sensors();
  Setpoint = 0;
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-MOTOR_MAX_SPEED + 105, MOTOR_MAX_SPEED - 105);
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
  Input =  2 - ( ( !ir1on + 2 * (ir2on) + 3 * !ir3on )  / ( !ir1on + ir2on + !ir3on) );
  myPID.Compute();
  speedA = cruiseSpeed + Output;
  speedB = cruiseSpeed - Output;
  motor(speedA, speedB);
}


void halt_motors() {
  isHalted = true;
  manualB = manualA = 0;
  myPID.SetMode(MANUAL);
  motor(0, 0);
}
void resume_motors() {
  isHalted = false;
  manualB = manualA = 0;
  myPID.SetMode(MANUAL);
  motor(0, 0);
}



void lf_pid_p(int16_t v) {
  //DBG("%d", v);
  Kp = ((double)v) / 100.0;
  myPID.SetTunings(Kp, Ki, Kd);

}
void lf_pid_i(int16_t v) {
  //DBG("%d", v);
  Ki = ((double)v) / 100.0;
  myPID.SetTunings(Kp, Ki, Kd);

}
void lf_pid_d(int16_t v) {
  //DBG("%d", v);
  Kd = ((double)v) / 100.0;
  myPID.SetTunings(Kp, Ki, Kd);

}


#endif


