//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Required Libraries - No need to change this section
#include <EEPROMex.h>
#include <Cannybots.h>
#include "CannybotsRacerGlu.h"
Cannybots& cb = Cannybots::getInstance();
cb_app_config settings;

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Bot State

// IR sensors values dark=0..~1000 =light, updated automatically.
int IRvals[NUM_IR_SENSORS];      

// Speed
// speeds are fullspeed back -255 .. full speed forward 255
int cruiseSpeed = 0;
int speedA = 0; // viewed from behind motor 'A' is on the left  and a +ve speed rotates the wheel in the forward direction (made to be true by config)
int speedB = 0; // viewed from behind motor 'B' is on the right and a +ve speed rotates the wheel in the forward direction (made to be true by config)

// Joystick
int xAxisValue = 0;  // (left) -255 .. 255 (right)
int yAxisValue = 0;  // (down) -255 .. 255 (up)

volatile bool isLineFollowingMode = true;       // set when the bot has detected it's on the line and thus being controlled by PID (always 'false' when forceManualMode is 'true') 
volatile bool forceManualMode     = false;          // when true the user is forcing manual mode from the phone app/joypad
volatile bool isTankControlMode   = false;        // when true the client has a left (xAxisValue) and right (yAxisValue) throttle, e.g. like tank control for independant left/right track speed controll.

// Lap Timing
unsigned long currentStartLapTime = 0;
int  lapCount=0;

// used for stats and logging
volatile unsigned long loopNowTime = millis();
volatile unsigned long loopDeltaTime = millis();

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// PID

int Kp = 0, Ki = 0, Kd = 0;
int P_error = 0, D_error = 0;
int error = 0;
int error_last = 0;                                     // to calculate D_error = error - error_last
int correction = 0;                                     // error after PID filter

void pid_calculate() {
  // get the values from config (these might be updated in the background by the user), or, override them here.
  Kp = settings.cfg_pid_p;
  Kd = settings.cfg_pid_d;
  cruiseSpeed = settings.cfg_cruiseSpeed_defaultSpeed;
  
  // process IR readings via PID
  error_last = error;                                   // store previous error before new one is caluclated
  error = IRvals[0] - IRvals[2];                        // TODO: change to lineFOllowingLib.getIRreading(IR_SENSORID);

  P_error = error * Kp / (float)settings.cfg_pid_divisor;                               // calculate proportional term
  D_error = (error - error_last) * Kd / (float)settings.cfg_pid_divisor;                // calculate differential term
  correction = P_error + D_error;
  speedA = cruiseSpeed + correction;
  speedB = cruiseSpeed - correction;
}

// This is called when the bot is on the line
// speedA and speedB will have already been set by pid_calculate() before this is run
void  joystick_lineFollowingControlMode() {
    speedA = speedA + (yAxisValue/3); //superpose yAxis with PID output speed
    speedB = speedB + (yAxisValue/3);
}

// This is called when the bot is in manual mode.
// Use it to map the joystick X & Y axis (which both range from -255..255) to motor speeds (also -255..255)
void joystick_manualControlMode() {
  if (isTankControlMode) {
    speedA = xAxisValue/4;
    speedB = yAxisValue/4;
  } else {
    speedA =  (yAxisValue + xAxisValue)/4; 
    speedB =  (yAxisValue - xAxisValue)/4; 
  }

}
//////////////////////////////////////////////////////////////////////////////////////////////////
// Lap Timing

// this should be called at least once to inform the client (e.g. phone) that the bot has started racing
void lap_started() {
  currentStartLapTime = loopNowTime;
  lineFollowingUtilities_lapStarted(currentStartLapTime, lapCount);
  lapCount++;
}

// call on lap complete, updates phone with current lap time & lap count, it then restarts the lap timer and incremetn the laptcount
void lap_completed() {
  lineFollowingUtilities_lapComplete(loopNowTime-currentStartLapTime, lapCount);
  currentStartLapTime = loopNowTime;
}

// this should be called to tell the client (e.g. phone app) that lap timing and counting has finished
void lap_stopTiming() {
  lineFollowingUtilities_stopLapCounting();
}

// Debugging

  
int testState = 0;
  
// this is called as often as 'settings.cfg_info_printValsInterval' specifies, in ms.
void print_debug() {  
  //TODO: fix mem leak in CB_DBG2REMOTE
  CB_DBG(    "%lu(%lu): IR(%u,%u,%u),Kpd(%d,%d)/100,Sab(%d,%d), XY(%d,%d),MEM(%d)",
             loopNowTime,
             loopDeltaTime,
             IRvals[0], IRvals[1], IRvals[2],
             Kp*100, Kd*100, 
             speedA, speedB,
             xAxisValue, yAxisValue,
             cb.getFreeMemory()
        );
   return;
  switch (testState) {
    case 1: lap_started();  break;
    case 2: lap_completed(); lap_started(); break;
    case 3: lap_completed(); lap_started(); break;
    case 4: lap_completed(); break;
    case 5: lap_stopTiming(); break;
  }
  testState = (testState +1 ) % 10; // do nothing for 4 'intervals' at end
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Normal Arduino Setup & Main Loop
void setup() {
  lineFollowingUtilities_setup();
  //cb.dumpConfig();
  CB_DBG("A*START!",0);
}


void loop() {
  lineFollowingUtilities_loop();
}

