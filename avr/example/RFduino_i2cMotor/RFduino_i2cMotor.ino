///////////////////////////////////////////////////////////////////////////////////

// based on: https://github.com/sparkfun/MiniMoto

// The default MiniMot library and example ported to RFduino

// Example sketch for driving 2 motor boards
// Wiring fro RFduino to BOTH breakout boards

// RFduino gpio 4 => drv8830 FAULT  
// RFduino gpio 5 => drv8830 SCL
// RFduino gpio 6 => drv8830 SCK

#define FAULTn  4     // Pin used for fault detection.

// The Motor driver I2C address is defined by pulling 2 pins (A0, A1) high or low or leaving them floating, it need to be unique per board.
// See the table on page 12 of the datasheet: http://dlnmh9ip6v2uc.cloudfront.net/datasheets/BreakoutBoards/drv8830.pdf
// 
#define MOTOR_A_ADDR  0xCE
#define MOTOR_B_ADDR  0xD0
// The above means:
// Motor A pins:  A0 not connected, A1 not connected
// Motor B pins:  A0 pulled high, A1 pulled high


// the original default sketch follows pretty much as it was (apart from an include and the _ADDR macros) ...

///////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include "minimoto.h"  // Include the MiniMoto library

// Create two MiniMoto instances, with different address settings.
MiniMoto motor0(MOTOR_A_ADDR); // A1 = 1, A0 = clear
MiniMoto motor1(MOTOR_B_ADDR); // A1 = 1, A0 = 1 (default)


// Nothing terribly special in the setup() function- prep the
//  serial port, print a little greeting, and set up our fault
//  pin as an input.
void setup()
{
  Serial.begin(9600);
  Serial.println("Hello, world!");
  pinMode(FAULTn, INPUT);
}

// The loop() function just spins the motors one way, then the
//  other, while constantly monitoring for any fault conditions
//  to occur. If a fault does occur, it will be reported over
//  the serial port, and then operation continues.
void loop()
{
  Serial.println("Forward!");
  motor0.drive(10);
  motor1.drive(10);
  delayUntil(10000);
  Serial.println("Stop!");
  motor0.stop();
  motor1.stop();
  delay(2000);
  Serial.println("Reverse!");
  motor0.drive(-10);
  motor1.drive(-10);
  delayUntil(10000);
  Serial.println("Brake!");
  motor0.brake();
  motor1.brake();
  delay(2000);
}

// delayUntil() is a little function to run the motor either for
//  a designated time OR until a fault occurs. Note that this is
//  a very simple demonstration; ideally, an interrupt would be
//  used to service faults rather than blocking the application
//  during motion and polling for faults.
void delayUntil(unsigned long elapsedTime)
{
  // See the "BlinkWithoutDelay" example for more details on how
  //  and why this loop works the way it does.
  unsigned long startTime = millis();
  while (startTime + elapsedTime > millis())
  {
    // If FAULTn goes low, a fault condition *may* exist. To be
    //  sure, we'll need to check the FAULT bit.
    if (digitalRead(FAULTn) == LOW)
    {
      // We're going to check both motors; the logic is the same
      //  for each...
      byte result = motor0.getFault();
      // If result masked by FAULT is non-zero, we've got a fault
      //  condition, and we should report it.
      if (result & FAULT)
      {
        Serial.print("Motor 0 fault: ");
        if (result & OCP) Serial.println("Chip overcurrent!");
        if (result & ILIMIT) Serial.println("Load current limit!");
        if (result & UVLO) Serial.println("Undervoltage!");
        if (result & OTS) Serial.println("Over temp!");
        break; // We want to break out of the motion immediately,
               //  so we can stop motion in response to our fault.
      }
      result = motor1.getFault();
      if (result & FAULT)
      {
        Serial.print("Motor 1 fault: ");
        if (result & OCP) Serial.println("Chip overcurrent!");
        if (result & ILIMIT) Serial.println("Load current limit!");
        if (result & UVLO) Serial.println("Undervoltage!");
        if (result & OTS) Serial.println("Over temp!");
        break;
      }
    }
  }
}
