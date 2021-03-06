#include <Arduino.h>
#include "minimoto.h"

// The address of the part is set by a jumper on the board. 
//  See datasheet or schematic for details; default is 0xD0.
MiniMoto::MiniMoto(byte addr)
{
  _addr = addr;
  
  // This sets the bit rate of the bus; I want 100kHz. See the
  //  datasheet for details on how I came up with this value.
  Wire.speed=100;
  
  Wire.begin();
}

// Return the fault status of the DRV8830 chip. Also clears any existing faults.
byte MiniMoto::getFault()
{
  byte buffer = 0;
  byte clearFault = 0x80;
  I2CReadBytes(0x01, &buffer, 1);
  I2CWriteBytes(0x01, &clearFault, 1);
  return buffer;
}

// Send the drive command over I2C to the DRV8830 chip. Bits 7:2 are the speed
//  setting; range is 0-63. Bits 1:0 are the mode setting:
//  - 00 = HI-Z
//  - 01 = Reverse
//  - 10 = Forward
//  - 11 = H-H (brake)
void MiniMoto::drive(int speed)
{
  byte regValue = 0x80;             // Before we do anything, we'll want to
                                    //  clear the fault status. To do that
                                    //  write 0x80 to register 0x01 on the
                                    //  DRV8830.
  I2CWriteBytes(0x01, &regValue, 1); // Clear the fault status.
  regValue = (byte)abs(speed);      // Find the byte-ish abs value of the input
  if (regValue > 63) regValue = 63; // Cap the value at 63.
  regValue = regValue<<2;           // Left shift to make room for bits 1:0
  if (speed < 0) regValue |= 0x01;  // Set bits 1:0 based on sign of input.
  else           regValue |= 0x02;
  
  I2CWriteBytes(0x00, &regValue, 1);  
}

// Coast to a stop by hi-z'ing the drivers.
void MiniMoto::stop()
{
  byte regValue = 0;                // See above for bit 1:0 explanation.
  
  I2CWriteBytes(0x00, &regValue, 1); 
}

// Stop the motor by providing a heavy load on it.
void MiniMoto::brake()
{
  byte regValue = 0x03;                // See above for bit 1:0 explanation.
  
  I2CWriteBytes(0x00, &regValue, 1); 
}

// Private function that reads some number of bytes from the accelerometer.
void MiniMoto::I2CReadBytes(byte addr, byte *buffer, byte len)
{
  Wire.requestFrom(addr, len);
  for (byte i = 0; i < len; i++)
  {   
    buffer[i] = Wire.read();
  }
}

void MiniMoto::I2CWriteBytes(byte addr, byte *buffer, byte len)
{
  Wire.beginTransmission(addr); 
  for (byte i = 0; i < len; i++) {
    Wire.write(buffer[i]);             
  }
  Wire.endTransmission();   
}
