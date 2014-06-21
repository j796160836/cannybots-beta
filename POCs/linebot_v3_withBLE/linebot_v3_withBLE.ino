/*
Linebot 1
for Arduino Nano

*/

#include <SPI.h>
// This lib had the .h and .cpp tweaked to disable debug and remove hard coded debug
#include <Adafruit_BLE_UART.h>
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
Adafruit_BLE_UART BLESerial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

void aciCallback(aci_evt_opcode_t event)
{
  switch (event)
  {
    case ACI_EVT_DEVICE_STARTED:
      break;
    case ACI_EVT_CONNECTED:
      break;
    case ACI_EVT_DISCONNECTED:
      break;
    default:
      break;
  }
}

void rxCallback(uint8_t *buffer, uint8_t len) {
}



// PIN Assignments
// IR sensor pins
const int IR0 = A0;
const int IR1 = A1;  // Analog input pin that the potentiometer is attached to
const int IR2 = A2; // Analog output pin that the LED is attached to
const int IR3 = A3;
const int IR4 = A4;

// motor control pins
const int enableA = 5; // analog input to control speed
const int phaseA = 4; //digital output to control direction
const int enableB = 6;
const int phaseB = 7;

// variable definitions
int IR1_val = 0; //reading from IR sensor 1
int IR2_val = 0;
int IR3_val = 0;
int mean_val = 0;
int error = 0;
//int white_mean = 22;
//int black_mean = 33;
int IR1_bias = 4;
int IR2_bias = 0;
int IR3_bias = -3;

int speedA = 0;
int speedB = 0;

// Gain setting
int speedScaling = 0;
int cruiseSpeed = 0;

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


void setup()
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(phaseA, OUTPUT);
  pinMode(phaseB, OUTPUT);
  
  BLESerial.setRXcallback(rxCallback);
  BLESerial.setACIcallback(aciCallback);
  BLESerial.begin();
}

void loop()
{
   BLESerial.pollACI();

  // read the IR sensors:
  IR1_val = analogRead(IR1);
  IR1_val = analogRead(IR1) - IR1_bias; //left looking from behind
  //set limit on reading. The reading can be very high and inaccurate on pitch black
  if (IR1_val > 100)
  IR1_val = 100;
  
  IR2_val = analogRead(IR2);
  IR2_val = analogRead(IR2) - IR2_bias; //centre
  if (IR2_val > 100)
  IR2_val = 100;
  
  IR3_val = analogRead(IR3);
  IR3_val = analogRead(IR3) - IR3_bias; //right
  if (IR3_val > 100)
  IR3_val = 100;

  // process IR readings
  error_last = error; //store previous error before new one is caluclated
  error = IR1_val - IR3_val;
  //set bounds for error
  if (error > 30)
    error = 30;
  if (error < -30)
    error = -30;

  Kp = 6;
  Ki = 0;
  Kd = 6;

  // calculate proportional term
  P_error = error * Kp;

  // calculate differential term
  D_error = (error - error_last)*Kd;

  correction = P_error + D_error;

  //if sernsor 2 is on white, set spped to zero
  if (IR2_val >= 100)
    cruiseSpeed = 100;
  else
  {
    cruiseSpeed = 0;
    correction = 0;
  }

  // Set motor speed
  // If correction is > 0, increase the speed of motor A and 
  //decrease speed of motor B. If correction is < 0,  
  // decrease speed of A and increase speedB.
  speedA = cruiseSpeed + correction;
  speedB = cruiseSpeed - correction;
   
  // limit max speed
  if (speedA > 150)
  speedA = 150;
  if (speedA < -150)
  speedA = -150;
  if (speedB > 150)
  speedB = 150;
  if (speedB < -150)
  speedB = -150;

  motor(speedA, speedB);


  // print values
  if (printdelay == 50) {
     printvalues();
    printdelay = 1;
  }
  else {
    printdelay = printdelay + 1;
  }
  //delay(10);
}

// motor controller function
void motor(int speedA, int speedB)
{
  if (speedA >= 0)
  digitalWrite(phaseA, HIGH);
  if (speedA <0)
  digitalWrite(phaseA, LOW);
  if (speedB >= 0)
  digitalWrite(phaseB, HIGH);
  if (speedB < 0)
  digitalWrite(phaseB, LOW);
  
  speedA = abs(speedA); //speed is unsigned
  speedB = abs(speedB);
  
  // upper cutoff
  if (speedA > 255)
    speedA = 255;
  if (speedB > 255)
    speedB = 255;

  analogWrite(enableA, (255 - speedA));
  analogWrite(enableB, (255 - speedB));

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


