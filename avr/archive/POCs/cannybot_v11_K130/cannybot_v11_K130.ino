/*
Cannybot v1
for Arduino Nano

*/

// PIN Assignments
// IR sensor pins
const int IR1 = A6; 
const int IR2 = A8; 
const int IR3 = A11; 

// motor control pins
const int pinA1 = 3; 
const int pinA2 = 5; 
const int pinB1 = 6;
const int pinB2 = 9;
const int pin_MODE = 2;

// variable definitions
int IR1_val = 0; //reading from IR sensor 1
int IR2_val = 0;
int IR3_val = 0;
int error = 0;

int IR1_bias = 0;
int IR2_bias = 0;
int IR3_bias = 0;

int globalSpeed = 0;
int speedA = 0;
int speedB = 0;

// Gain setting
int speedScaling = 0;
int cruiseSpeed = 0;

int stopCounter = 0;

int Kp = 0;
int Ki = 0;
int Kd = 0;
int I_limit = 100;

int P_error = 0;
int I_error = 0;
int D_error = 0;
int error_last = 0; // to calculate D_error = error - error_last
int correction = 0; //error after PID filter
int printdelay = 1; //counter to slow print rate

int whiteflag = 0;

void setup()
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
    pinMode(pinA1, OUTPUT);
    pinMode(pinA2, OUTPUT);
    pinMode(pinB1, OUTPUT);
    pinMode(pinB2, OUTPUT);
    pinMode(pin_MODE, OUTPUT);
    //pinMode(13,OUTPUT);
  
  digitalWrite(pin_MODE, HIGH); //to set controller to Phase/Enable mode
  digitalWrite(13,HIGH); // turn on the LED 
}

void loop()
{

  // read the IR sensors:
  IR1_val = (analogRead(IR1) - IR1_bias); //left looking from behind
  IR2_val = (analogRead(IR2) - IR2_bias); //centre
  IR3_val = (analogRead(IR3) - IR3_bias); //right
  
  // process IR readings
  error_last = error; //store previous error before new one is caluclated
  error = IR1_val - IR3_val;
  
  Kp = 20;
  Ki = 0;
  Kd = 50;

  // calculate proportional term
  P_error = error * Kp/100;
  // calculate differential term
  D_error = (error - error_last)*Kd/100;

  correction = (P_error + D_error);
  
  //if sernsor 2 is on white, set spped to zero
  if (IR2_val >= 700) 
  {
    cruiseSpeed = 180;
  }
    else
  {
    cruiseSpeed = 0;
    correction = 0;
  }
 

  // Set motor speed
  // If correction is > 0, increase the speed of motor A and 
  //decrease speed of motor B. If correction is < 0,  
  // decrease speed of A and increase speedB.
  speedA = (cruiseSpeed + correction);
  speedB = (cruiseSpeed - correction);
  
  motor(speedA, -speedB);


 // print values
  if (printdelay == 50) {
     printvalues();
    printdelay = 1;
  }
  else {
    printdelay = printdelay + 1;
  }

delay(10);
}

// motor controller function
void motor(int speedA, int speedB)
{
 
  if (speedA >= 0){
    if (speedA > 255)
    speedA = 255;
    digitalWrite(pinA1,HIGH);
    analogWrite(pinA2, speedA);
  }
  
  if (speedA < 0){
   if (speedA < -255)
    speedA = -255; 
    digitalWrite(pinA1,LOW);
    analogWrite(pinA2,-speedA);
  }
  
  if (speedB >= 0){
    if (speedB > 255)
    speedB = 255;
   digitalWrite(pinB1, LOW);
   analogWrite(pinB2,speedB);
  }
  
  if (speedB < 0){
   if (speedB < -255)
    speedB = -255; 
    digitalWrite(pinB1,HIGH);
    analogWrite(pinB2,-speedB);
  }
  
}

void printvalues ()
{

  Serial.print(IR1_val);
  Serial.print(", ");
  Serial.print(IR2_val);
  Serial.print(", ");
  Serial.print(IR3_val);
  Serial.print(", ");
  Serial.print(error);
  Serial.print(", ");
  Serial.print(P_error);
  Serial.print(", ");
  Serial.print(D_error);
  Serial.print(", ");
  Serial.print(speedA);
  Serial.print(", ");
  Serial.println(speedB);
}


