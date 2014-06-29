// Platform include
#include <Cannybots.h>

// This enum  represents the ID's for variables that need to be 
// handled (passed between client and bot) by the Cannybots library
enum RacerCommands { 
  RACER_KP,
  RACER_KI,
  RACER_KD,
  RACER_SET_SPEED,
  RACER_IRBIAS,
  RACER_BASECRUISESPEED,
  RACER_CURRENTCRUISESPEED,
  RACER_LINEFOLLOWING_MODE,
  RACER_MAX_COMMAND_ID
};
// other ID's for variables/settings like DeviceID, serial number, battery power, 
// radio power & signal strength, would be pre-defined by the Cannybots library so 
// it was commonly known across all apps & bots.



// User application settings that Cannybots library will be told about
#define NUM_MOTORS       2
#define NUM_IR_SENSORS   3

int  Kp, Ki, Kd;
bool isLineFollowingMode;
int  speeds[NUM_MOTORS];
int  IRbias[NUM_IR_SENSORS];
int  baseCruiseSpeed, currentCruiseSpeed;


// The Cannybots object which handles bi-directional data passing between the Sketch and the 'client'
Cannybots cb();

// The Cannybots setup that allows variable to be set and read remotely.
void cannybots_setup() {
  
  cb.setEeprom(F("CBLF"), 128);  // optional
  // the paramters are an EEPROM magic marker and an EEPROM offset
  // The marker is checked to see if the app EEPROM config already exists; it will be set to defaults if not
  // The second parameter is the start offset into EEPROM when the config should be looked for first
  // An EEPROM search for the magic marker maybe performed if not found at that specifc location)
  // if the app doesnt need EEPROM settings dont call this function

  // Step 1:  
  // Registar simple variables that can be set and read remotely by the client
  
  // Step 1.1: 
  // register variables that can be saved to EEPROM
  // Function parameters are : enum ID, variable address, is saved in EEPROM?, default if EEPROM not setup.
  cb.registerVariable_int16(RACER_KP, &Kp, true, 0); 
  cb.registerVariable_int16(RACER_KD, &Kd, true, 0); 
  cb.registerVariable_int16(RACER_BASECRUISESPEED, &baseCruiseSpeed, true, 0); 
  cb.registerVariable_bool(RACER_LINEFOLLOWING_MODE, &isLineFollowingMode, true, true); 
  
  // Step 1.2: 
  // Register variables that don't need to be saved to EEPROM
  
  cb.registerVariable_int16(RACER_CURRENTCRUISESPEED, &currentCruiseSpeed); 
  
  
  // Step 2
  // Register arrays that can be set and read remotely by the client
  
  // Step 2.1
  // parameters are : ID, array address, array length
  cb.registerArray_int16(RACER_IRBIAS, &IRbias, NUM_IR_SENSORS); 
  
  
  // Step 3
  // Register variabls that 'push' values from the bot to the clien

  // Function parameters are : enum ID, variable address, update method which is one of:
  // CB_PUB_UPDATE_ONCHANGE - only send an update when the value changes
  // CB_PUB_UPDATE_ALWAYS   - send an update each time cb.update() is called (e.g. once per loop() )
  cb.registerPublisher(RACER_LINEFOLLOWING_MODE, isLineFollowingMode, CB_PUB_UPDATE_ONCHANGE);
  // maybe todo, sub modifiers with optional values, e.g:
  // registerPublisher(..., CB_PUB_UPDATE_DEBOUNCED, CB_PUB_UPDATE_LESSTHAN, 100);
  // registerPublisher(..., CB_PUB_UPDATE_DEBOUNCED, CB_PUB_UPDATE_GREATERTHAN, 200);
  // registerPublisher(..., CB_PUB_UPDATE_ALWAYS, IN_RANGE, 150,200);
  // registerPublisher(..., CB_PUB_UPDATE_ALWAYS, OUTSIDE_RANGE, 150,200);
  
  
  // Step 4 
  // Register functions that can be called by the client
  cb.registerHandler_int16_x2(RACER_SET_SPEED, lf_setSpeed);
  
  
  // Step 5:
  // expose some variable in the client scripting language 
  // params are: variable id, variable name to use in script
  cb.registerScritableVariable(RACER_CURRENTCRUISESPEED, F("currentCruiseSpeed"));


  // Step 6:
  // Configure the client side custom GUI tab
  // avaialbe GUI elemetns are: buttons, level meters, x & y sliders (get&set), joystick, ...
  // x, y, button lable, function to call in sketch when pressed
  cb.gui_addButton(50,100, F("Stop"), lf_gui_stopPressed);
  cb.gui_addButton(250,100, F("Go"), lf_gui_goPressed);
  // x, y,variable to send to client, var range min value, var range max value
  cb.gui_addLevelMeter(100,200, &currentCruiseSpeed, 0, 255);
  
  
  if (!cb.validate()) {
    // error in setup, e.g. duplicate ID's used
    Serial.println(cb.lastError());
    // Flash some lights, ring some alarms :)
  }
  
  // optionally: cb.debug(true, Serial);
}

void lf_setSpeed(int speedA, int speedB) {
   // Do motor speed settings
}

void setup() {
  cannybots_setup();
  
  // Do 'normal' application/user sketch setup
  
  // LINE FOLLOWING SETUP CODE GOES HERE... (e.g. set output pins)

}

void loop() {
  cb.update();
  // the above call will:
  // 1. poll queues and execute commands, e.g. set/get variables and/or save to EEPROM, invoke methods
  // 2. send variable values that need sending to client

  // Do 'noraml' application/user sketch loop code
  
  // LINE FOLLOWING CODE GOES HERE...  (e.g. do the PID wiggle)
}


