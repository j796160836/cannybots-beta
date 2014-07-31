#include <Arduino.h>
#include "NT_MOD_SensorCommon.h"
#include "NTProtocol.h" 
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
int samplingInterval = 200;          // how often to run the main loop (in ms)


void emitSensorData() {
  // TODO: read from EEPROM config what to send.
#ifdef USE_SONAR
  sonar_generateReadings(0); // 0 =all,  1 = 1st, 2 = 2nd
#endif
#ifdef USE_IRRECV
  irrecv_generateReadings(0); // 0 =all,  1 = 1st, 2 = 2nd

#endif

#ifdef USE_MIC
  mic_generateReadings(0); // 0 =all,  1 = 1st, 2 = 2nd
#endif
}


void sensor_update() {
  int analogreportnums = ANALOG_REPORT_NUM;
  samplingInterval = (uint16_t)MINIMUM_SAMPLE_DELAY  + (uint16_t)ANALOG_SAMPLE_DELAY * (1 + analogreportnums);
  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    emitSensorData();
  }
  
}
