//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots LapCounter GZLL
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0   -  27.08.2014  -  Inital Version
//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <RFduinoGZLL.h>

#define NUM_RACERS 2

#define BOT1_HOST_BASE 0x91827332
#define BOT2_HOST_BASE 0x91827364

uint32_t hostIds[NUM_RACERS] = {
  BOT1_HOST_BASE,
  BOT2_HOST_BASE
};


// this is used by the bot RFduiono sketch to indicate this is a laptimer
device_t role = DEVICE2;
char msg[9] = {0};

int lapTimes[NUM_RACERS] = {0};
int laps[NUM_RACERS]     = {0};
int lastLaps[NUM_RACERS]  = {0};


void setup()
{
  Serial.begin(9600);
  snprintf(msg, sizeof(msg), "%c%c%5.5s%c", 0, 0, "LAPRQ", 0);
}

void startHost(uint32_t hostBaseAddress) {
  RFduinoGZLL.end();
  RFduinoGZLL.hostBaseAddress = hostBaseAddress;
  RFduinoGZLL.begin(role);
}

int hostIdIndex = 0;


unsigned long lastToggle = millis();
void loop()
{
  if ( (millis() - lastToggle) > 2000) {
    startHost(hostIds[hostIdIndex]);
    hostIdIndex  = (hostIdIndex + 1) % 2;
    lastToggle = millis();
    //Serial.println(hostIdIndex);
  }

  RFduinoGZLL.sendToHost(msg, 8);   // don't bother sending the NULL byte at the end
  delay(20);

  if (lastLaps[hostIdIndex] != laps[hostIdIndex]) {
    Serial.print("Racer:");
    Serial.print(hostIdIndex+1);
    Serial.print(" : ");
    printStats(lapTimes[hostIdIndex], laps[hostIdIndex]);
    lastLaps[hostIdIndex] = laps[hostIdIndex];
  }
}

void printStats(int _time, int _laps) {
  Serial.print("Lap ");
  Serial.print(_laps);
  Serial.print(", time=");
  Serial.println(_time / 1000.0, 2);
}


void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  if (len >= 12) {
    int index = data[1];
    lapTimes[index] = (data[8] << 8)  + data[9];
    laps[index] = (data[10] << 8) + data[11];
  }
}
