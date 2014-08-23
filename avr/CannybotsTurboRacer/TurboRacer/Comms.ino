//////////////////////////////////////////////////////////////////////////////////////////////////
// Message Handling


//////////////////////////////////////////////////////////////////////////////////////////////////
// Serial Comms

// Serial Input
// We're expecting messages of 20 bytes in the form:  >>{id}{name}[bytes]
// Where
// >> = start marker, as-is
// id   (1 byte)    = client ID  (e.g. 0-7 for GZLL joypad ID's, 10-255 for anything else)
// name (5 bytes)   = variable name
// bytes (0..14 bytes) = binary payload

void readSerial() {
  static int c = 0, lastChar = 0;
  char   varName[6]    = {0};                    // 5  + 1 null

  while (Serial1.available() >= 20) {
    lastChar = c;
    c = Serial1.read();
    if ( ('>' == c) && ('>' == lastChar) ) {

      byte id =  Serial1.read();
      int  bytesRemaining = 20-6;          // we have to consume any unused serial data of the fixed 20 bytes
      
      if (id == JOYPAD_ID) {
        // read bytes 1..5 = Var name
        for (int i = 0 ; i < 5; i++) {
          varName[i] = Serial1.read();
        }
        varName[5] = 0;
        updateVariable(varName, &bytesRemaining);
#if 0
        Serial.print("id=");
        Serial.print(id);
        Serial.print(",name=");
        Serial.print(varName);
#endif
      }
      // consume and ignore any remaining bytes
      while (bytesRemaining--) {
        Serial1.read();
      }
      
      lastChar = c = 0;
      joypadLastTime = timeNow;                      // record the time we last received a joypad command.
    }
  }
}

// Reading data

int readInt(int* count) {
  int v1 = Serial1.read();
  int v2 = Serial1.read();   
  *count-=2;
  return  (v1<<8) | (v2 & 0xFF);
}

// Writing data

#define SERIAL_MESSAGE_BUFFER_SIZE 23             // 20 byte paload + 2 byte start marker (>>) + 1 null terminator used during string creation (not sent over serial)

void writeData(const char* name, int16_t p1) {
  char msg[SERIAL_MESSAGE_BUFFER_SIZE] = {0}; 
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c%c", BOT_ID, name, highByte(p1), lowByte(p1));
  Serial1.write(msg,sizeof(msg)-1);
}

void writeData(const char* name, int16_t p1,  int16_t p2) {
  char msg[SERIAL_MESSAGE_BUFFER_SIZE] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c%c%c%c", BOT_ID, name, highByte(p1), lowByte(p1), highByte(p1), lowByte(p2));
  Serial1.write(msg,sizeof(msg)-1);
}

void writeData(const char* name, int16_t p1,  int16_t p2,  int16_t p3) {
  char msg[SERIAL_MESSAGE_BUFFER_SIZE] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c%c%c%c%c%c", BOT_ID, name, highByte(p1), lowByte(p1), highByte(p1), lowByte(p2), highByte(p3), lowByte(p3));
  Serial1.write(msg,sizeof(msg)-1);
}


