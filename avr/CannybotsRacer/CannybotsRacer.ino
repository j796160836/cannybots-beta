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
int cruiseSpeed = settings.cfg_cruiseSpeed_defaultSpeed;
int speedA = 0, speedB = 0;

// Joystick
int yAxisValue = 0;  // -255..255
int xAxisValue = 0;  // -255..255

bool forceManualMode = false;          // when true the user is forcing manual mode from the phone app/joypad

// Lap Timing
unsigned long currentStartLapTime = 0;
int  lapCount=0;

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// PID

int Kp = 0, Ki = 0, Kd = 0;
int P_error = 0, D_error = 0;
int error = 0;
int error_last = 0;                                     // to calculate D_error = error - error_last
int correction = 0;                                     // error after PID filter

void pid_calculate() {
  // Note: to override config just set 'Kp' and 'Kd' here, for example.
  
  // process IR readings via PID
  error_last = error;                                   // store previous error before new one is caluclated
  error = IRvals[0] - IRvals[2];                        // TODO: change to lineFOllowingLib.getIRreading(IR_SENSORID);

  P_error = error * Kp / settings.cfg_pid_divisor;                               // calculate proportional term
  D_error = (error - error_last) * Kd / settings.cfg_pid_divisor;                // calculate differential term
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
    speedA =  (yAxisValue + xAxisValue)/4; //-xAxisValue
    speedB =  (yAxisValue - xAxisValue)/4; //xAxisValue
}
//////////////////////////////////////////////////////////////////////////////////////////////////
// Lap Timing

// this should be called at least once to inform the phone that the bot has started racing
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

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Normal Arduino Setup & Main Loop

void setup() {
  lineFollowing_setup();
  cannybots_setup(); 
}

void loop() {
  lineFollowing_loop();
  cb.update();
}

