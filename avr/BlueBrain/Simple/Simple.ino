#include <RFduinoGZLL.h>
#include <RFduinoBLE.h>

// Motor Pins
#define MOTOR_A1_PIN                 0
#define MOTOR_A2_PIN                 6  

#define MOTOR_B1_PIN                 1
#define MOTOR_B2_PIN                 5

#define MOTOR_MAX_SPEED            255
#define JOYPAD_AXIS_DEADZONE 10

#define LASER_PIN 4
#define PAN_PIN   3
#define TILT_PIN  2

// Joypad
volatile int16_t  xAxisValue    = 0;              // (left) -255 .. 255 (right)
volatile int16_t  yAxisValue    = 0;              // (down) -255 .. 255 (up)
volatile bool     buttonPressed = 0;              // 0 = not pressed, 1 = pressed

unsigned long timeNow = millis();                    // the time at the start of the loop()
#include <Servo.h> 

//Servo servos[2]; 
void setup() {
  Serial.end();

  // Motor pins
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  /*
  pinMode(PAN_PIN, OUTPUT);
  pinMode(TILT_PIN, OUTPUT);
  motorSpeed(0, 0);
  digitalWrite(LASER_PIN,HIGH);

  servos[0].attach(PAN_PIN);
  servos[1].attach(TILT_PIN);
  for(int pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    servos[0].write(pos);           // sets the servo 1 position according to the scaled value 
    servos[1].write(pos);           // sets the servo 2 position according to the scaled value 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  */
  
  //motorTest(255,2000,500);
  radio_setup();
}

void loop() {
  radio_loop();    
  motorSpeed(yAxisValue, xAxisValue);
  digitalWrite(LASER_PIN,buttonPressed?LOW:HIGH);

}

void motorSpeed(int _speedA, int _speedB) {
  _speedA = constrain(_speedA, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  _speedB = constrain(_speedB, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

  digitalWrite(MOTOR_A1_PIN, _speedA >= 0 ? HIGH : LOW) ;
  analogWrite (MOTOR_A2_PIN, abs(_speedA));

  digitalWrite(MOTOR_B1_PIN, _speedB >= 0 ? HIGH : LOW);
  analogWrite (MOTOR_B2_PIN, abs(_speedB));
}


void joypad_update(int x, int y, int b) {
  // If the axis readings are small set them to 0
  if ( abs(x) < JOYPAD_AXIS_DEADZONE)  x = 0;
  if ( abs(y) < JOYPAD_AXIS_DEADZONE)  y = 0;

  xAxisValue = x;
  yAxisValue = y;
  //xAxisValue = (y+x)/2;
  //yAxisValue = (y-x)/2;
  buttonPressed = b;
}

void motorTest2(int maxSpeed, int onTime, int intTime) {
  // individual tests
  motorSpeed(maxSpeed, 0);   delay(onTime);     // dir Right
  motorSpeed(0, 0);     delay(intTime);
  motorSpeed(-maxSpeed, 0);   delay(onTime);    // dir Right
  motorSpeed(0, 0);     delay(intTime);
  motorSpeed(0, maxSpeed);   delay(onTime);     // back
  motorSpeed(0, 0);     delay(intTime);
  motorSpeed(0, -maxSpeed);  delay(onTime);     // back
  motorSpeed(0, 0);
  delay(intTime*3);
  
  // combined tests
  motorSpeed(maxSpeed, maxSpeed);   delay(onTime);    // dir right & back
  motorSpeed(0, 0);     delay(intTime);
  motorSpeed(-maxSpeed, -maxSpeed);   delay(onTime);  // left, fwd
  motorSpeed(0, 0);     delay(intTime);
  motorSpeed(-maxSpeed, maxSpeed);   delay(onTime);   // dir r,  back
  motorSpeed(0, 0);     delay(intTime);
  motorSpeed(maxSpeed, -maxSpeed);   delay(onTime);   // left, back
  motorSpeed(0, 0);     delay(intTime);

}
