

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Pixy Commands





#ifdef USE_PIXY
#include <Wire.h>
#include <PixyI2C.h>
PixyI2C pixy;

void setup_pixy() {
}

void updatePixy() {
  debug(F("Detected %d"), pixy.getBlocks());
}

#endif



