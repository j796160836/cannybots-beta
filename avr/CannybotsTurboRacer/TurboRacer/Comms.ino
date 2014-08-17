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
        bytesRemaining-=updateVariable(varName);
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

int readInt() {
  return word(Serial1.read(), Serial1.read());
}



// Writing data

void writeData(const char* name, int16_t value) {
  char msg[20 + 1] = {0};
  snprintf(msg, sizeof(msg), ">>%c%5.5s%c% .10d  ", BOT_ID, name, 'd', value);
  //Serial.write(msg, sizeof(msg));
  //Serial.println();
  Serial1.write(msg, sizeof(msg));
}

