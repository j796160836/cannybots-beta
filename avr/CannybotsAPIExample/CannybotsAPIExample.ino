 // Platform include
#include <Cannybots.h>

// These represents the ID's for variables that need to be 
// handled (passed between client and bot) by the Cannybots library, therefore they must map!!!
CB_ATTRIBUTE_ID(RACER_KP, 1, "Kp");
CB_ATTRIBUTE_ID(RACER_KI, 2, "Ki");
CB_ATTRIBUTE_ID(RACER_KD, 3, "Kd");
CB_ATTRIBUTE_ID(RACER_IRBIAS, 4, "IRbias");
CB_ATTRIBUTE_ID(RACER_BASECRUISESPEED, 5, "baseCruiseSpeed");
CB_ATTRIBUTE_ID(RACER_CURRENTCRUISESPEED, 6, "currentCruiseSpeed");
CB_ATTRIBUTE_ID(RACER_LINEFOLLOWING_MODE, 7, "isLineFollowingMode");

// may want a _METHOD_ID instead?
CB_ATTRIBUTE_ID(RACER_SET_SPEED, 8, "setMotorSpeeds");

// perhaps instead use a type-safe enum that is supported by the compiler's C++ lang spec, or see http://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Type_Safe_Enum

// other ID's for variables/settings like DeviceID, serial number, battery power, 
// radio power & signal strength, would be pre-defined by the Cannybots library so 
// it was commonly known across all apps & bots.



// User application settings that Cannybots library will be told about
#define NUM_MOTORS       2
#define NUM_IR_SENSORS   3

int  Kp, Ki, Kd;
bool isLineFollowingMode;
int  speeds[NUM_MOTORS];
int16_t  IRbias[NUM_IR_SENSORS];
int  baseCruiseSpeed, currentCruiseSpeed;


// The Cannybots object which handles bi-directional data passing between the Sketch and the 'client'
Cannybots cb;

// The Cannybots setup that allows variable to be set and read remotely.
void cannybots_setup() {
  
  cb.setConfigStorage("CBLF", 64);
  // the paramters are an EEPROM magic marker and an EEPROM offset
  // The marker is checked to see if the app EEPROM config already exists; values will be set to defaults if not
  // The second parameter is the start offset into EEPROM when the config should be looked for first
  // if the app doesnt need EEPROM settings dont call this function

  // by default when variables are updated and they are stored in EEPROM then a call to
  // cb.saveEEPROM() needs to be made excplicitly (save on EEPROM/Flash writes)

  // Step 1:  
  // Registar simple variables that can be set and read remotely by the client
  
  // Step 1.1: 
  // register variables that can be saved to EEPROM
  // Function parameters are : enum ID, variable address, is saved in EEPROM?, default if EEPROM not setup.
  cb.registerVariable(RACER_KP, &Kp, true, 0); 
  cb.registerVariable(RACER_KD, &Kd, true, 0); 
  cb.registerVariable(RACER_BASECRUISESPEED, &baseCruiseSpeed, true, 0); 
  cb.registerVariable(RACER_LINEFOLLOWING_MODE, &isLineFollowingMode, true, true); 
  
  // Step 1.2: 
  // Register variables that don't need to be saved to EEPROM
  
  // just make all variable scriptable
  //cb.registerVariable(RACER_CURRENTCRUISESPEED, &currentCruiseSpeed); 
  
  
  // Step 2
  // Register arrays that can be set and read remotely by the client
  
  // Step 2.1
  // parameters are : ID, array address, array length
  cb.registerArray(RACER_IRBIAS, IRbias, NUM_IR_SENSORS); 
  
  
  // Step 3
  // Register variabls that 'push' values from the bot to the clien

  // Function parameters are : enum ID, variable address, update method which is one of:
  // CB_PUB_UPDATE_ONCHANGE - only send an update when the value changes
  // CB_PUB_UPDATE_ALWAYS   - send an update each time cb.update() is called (e.g. once per loop() )
  cb.registerPublisher(RACER_LINEFOLLOWING_MODE, &isLineFollowingMode, Cannybots::PUBLISH_UPDATE_ONCHANGE);
  // maybe todo, sub modifiers with optional values, e.g:
  // registerPublisher(..., CB_PUB_UPDATE_DEBOUNCED, CB_PUB_UPDATE_LESSTHAN, 100);
  // registerPublisher(..., CB_PUB_UPDATE_DEBOUNCED, CB_PUB_UPDATE_GREATERTHAN, 200);
  // registerPublisher(..., CB_PUB_UPDATE_ALWAYS, IN_RANGE, 150,200);
  // registerPublisher(..., CB_PUB_UPDATE_ALWAYS, OUTSIDE_RANGE, 150,200);
  
  
  // Step 4 
  // Register functions that can be called by the client
  cb.registerHandler(RACER_SET_SPEED, lf_setMotorSpeeds);
  
  
  // Step 5:
  // expose some variable in the client scripting language 
  // params are: variable id, variable name to use in script
  cb.registerScritableVariable(RACER_CURRENTCRUISESPEED, F("currentCruiseSpeed"));


  // Step 6:
  // Configure the client side custom GUI tab
  // avaialbe GUI elemetns are: buttons, level meters, x & y sliders (get&set), joystick, ...
  // x, y, button lable, function to call in sketch when pressed
  cb.gui_addButton(50,100, "Stop", lf_gui_stopPressed);
  cb.gui_addButton(250,100, "Go", lf_gui_goPressed);
  // x, y,variable to send to client, var range min value, var range max value
  cb.gui_addLevelMeter(100,200, "Speed", &currentCruiseSpeed, 0, 255);
  
  
  if (!cb.validate()) {
    // error in setup, e.g. duplicate ID's used
    Serial.println(cb.getLastError());
    // Flash some lights, ring some alarms :)
  }
  
  // optionally: cb.debug(true, Serial);
  cb.begin();
}

void lf_setMotorSpeeds(int speedA, int speedB) {
   // Do motor speed settings
}

void setup() {
#ifdef ARDUINO_AVR_A_STAR_32U4
  // brownout detection reporting
  // from:  http://www.pololu.com/docs/0J61/7
  pinMode(13, OUTPUT);
  if (MCUSR & (1 << BORF))
  {
    // A brownout reset occurred.  Blink the LED
    // quickly for 5 seconds.
    for (uint8_t i = 0; i < 20; i++)
    {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
  }
  MCUSR = 0;
#else
#error not comiling using the Polulu A* bootload, are u sure you dont want brownout detection? if so comment this line
#endif
  
  cannybots_setup();
  
  // Do 'normal' application/user sketch setup
  
  // LINE FOLLOWING SETUP CODE GOES HERE... (e.g. set output pins)

}
void lf_gui_goPressed() {
}

void lf_gui_stopPressed() {
}

void loop() {
  cb.update();
  // the above call will:
  // 1. poll queues and execute commands, e.g. set/get variables and/or save to EEPROM, invoke methods
  // 2. send variable values that need sending to client

  // Do 'noraml' application/user sketch loop code
  
  // LINE FOLLOWING CODE GOES HERE...  (e.g. do the PID wiggle)
}


