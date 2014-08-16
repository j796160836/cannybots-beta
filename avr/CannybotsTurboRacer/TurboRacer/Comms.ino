//////////////////////////////////////////////////////////////////////////////////////////////////
// Message Handling


//////////////////////////////////////////////////////////////////////////////////////////////////
// Serial Comms

// Serial Input
// We're expecting messages of 20 bytes in the form:  >>[id][name][type][value]
// Where
// >> = start marker, as-is
// id   (1 byte)    = client ID  (e.g. 0-7 for GZLL joypad ID's, 10-255 for anything else)
// name (5 bytes)   = variable name
// type (1 byte)    = enum  ('b','B','d','D','l','L') =  (int8, uint8, int16, uint16, int32, uint32)
// value (14 bytes) = string representation of the variable (padded with spaces or 0's)
void readSerial() {
  static int c = 0, lastChar = 0;
  char   varName[6]    = {0};    // 5  + 1 null
  char   varData[12]   = {0};   // 10 + 1 null

  while (Serial1.available() >= 20) {
    lastChar = c;
    c = Serial1.read();
    if ( ('>' == c) && ('>' == lastChar) ) {

      // read byte 1 = ID
      byte id =  Serial1.read();

      // read bytes 2..6 = Var name
      for (int i = 0 ; i < 5; i++) {
        varName[i] = Serial1.read();
      }
      varName[5] = 0;
      // read byte 7 = Var Type
      byte varType = Serial1.read();


      // read bytes 8..20 = Var data
      for (int i = 0 ; i < 12; i++) {
        varData[i] = Serial1.read();
      }
      varData[11] = 0;
#if 0      
      Serial.print("id=");
      Serial.print(id);
      Serial.print(",name=");
      Serial.print(varName);
      Serial.print(",type=");
      Serial.print(varType);
      Serial.print(",data=");
      Serial.println(varData);
#endif    
      if (checkClientId(id)) {
        updateVariable(varName, varData);
      }
      lastChar = c = 0;
      joypadLastTime = timeNow;                      // record the time we last received a joypad command.
    }
  }
}

bool checkClientId(byte id) {
  return (id == JOYPAD_ID);
}

// Reading data

int readInt(const char* data) {
  int value = atoi(data);
  return value;
}

bool readBool(const char* data) {
  int value = atoi(data);
  return value != 0; 
}


// Writing data

void writeData(const char* name, int8_t value) {
  char msg[20] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c% .10d  ", BOT_ID, name, 'b', value);
  Serial1.write(msg, sizeof(msg));
}

void writeData(const char* name, uint8_t value) {
  char msg[20] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c% .10d  ", BOT_ID, name, 'B', value);
  Serial1.write(msg, sizeof(msg));
}

void writeData(const char* name, uint16_t value) {
  char msg[20] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c% .10d  ", BOT_ID, name, 'D', value);
  Serial1.write(msg, sizeof(msg));
}

void writeData(const char* name, int16_t value) {
  char msg[20] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c% .10d  ", BOT_ID, name, 'd', value);
  Serial1.write(msg, sizeof(msg));
}

void writeData(const char* name, int32_t value) {
  char msg[20] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c% .6ld  ", BOT_ID, name, 'L', value);
  Serial1.write(msg, sizeof(msg));
}

void writeData(const char* name, uint32_t value) {
  char msg[20] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c% .6lu  ", BOT_ID, name, 'l', value);
  Serial1.write(msg, sizeof(msg));
}


