//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Required Libraries - No need to change this section
#include <EEPROMex.h>
#include <Cannybots.h>
#include "CannybotsRacer.h"
Cannybots& cb = Cannybots::getInstance();
cb_app_config settings;

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// App State
// IR sensors values dark=0..~1000 =light, updated automatically.
int IRvals[NUM_IR_SENSORS];      

// Speed
// speeds are fullspeed back -255 .. 255 full speed forward 
int cruiseSpeed = settings.cfg_cruiseSpeed_defaultSpeed;
int speedA = 0; // viewed from behind motor 'A' is on the left  and a +ve speed rotates the wheel in the forward direction (made to be true by config)
int speedB = 0; // viewed from behind motor 'B' is on the right and a +ve speed rotates the wheel in the forward direction (made to be true by config)

// Joystick
int xAxisValue = 0;  // (left) -255 .. 255 (right)
int yAxisValue = 0;  // (down) -255 .. 255 (up)

bool isLineFollowingMode = true;       // set when the bot has detected it's on the line and thus being controlled by PID (always 'false' when forceManualMode is 'true') 
bool forceManualMode = false;          // when true the user is forcing manual mode from the phone app/joypad
bool isTankControlMode = false;        // when true the client has a left (xAxisValue) and right (yAxisValue) throttle, e.g. like tank ontrol for independant left/right track speed controll.

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
  Kp = settings.cfg_pid_p;
  Kd = settings.cfg_pid_d;
  
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
    speedA =  xAxisValue/4;
    speedB =  yAxisValue/4;
  } else {
    speedA =  (yAxisValue + xAxisValue)/4; 
    speedB =  (yAxisValue - xAxisValue)/4; 
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////
// Lap Timing

// this should be called at least once to inform the client (e.g. phone) that the bot has started racing
void lap_started() {
  currentStartLapTime = millis();
  cb.callMethod(&LAPCOUNTER_GETREADY, currentStartLapTime); 
}

// call on lap complete, updates phone with current lap time, lap count and restarts the lap timer
void lap_completed() {
  cb.callMethod(&LAPCOUNTER_LAPTIME, millis()-currentStartLapTime);
  cb.callMethod(&LAPCOUNTER_LAPCOUNT, lapCount);
  currentStartLapTime = millis();
  lapCount++;
}

// this should be called to tell the phone that lap timing has finished
void lap_stopTiming() {
  cb.callMethod(&LAPCOUNTER_STOP, lapCount);
}

// Debugging

// this is called as often as 'settings.cfg_info_printValsInterval' specifies, in ms.
void print_debug() {
  
  CB_DBG(    "%lu(%lu): IR(%u,%u,%u),Kpd(%d,%d)/100,Sab(%d,%d), XY(%d,%d),MEM(%d)\n",
             loopNowTime,
             loopDeltaTime,
             IRvals[0], IRvals[1], IRvals[2],
             Kp*100, Kd*100, 
             speedA, speedB,
             xAxisValue, yAxisValue,
             cb.getFreeMemory()
        );
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Normal Arduino Setup & Main Loop
void setup() {
  delay(2000);
  cannybots_setup(); 
  cb.dumpConfig();
  lineFollowing_setup();
}

void loop() {
  delay(100);
  cb.update();
  lineFollowing_loop();
}

