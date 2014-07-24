/*
Linebot 1

*/

// PIN Assignments
// IR sensor pins
const int IR1 = A0;  // Analog input pin that the potentiometer is attached to
const int IR2 = A1; // Analog output pin that the LED is attached to
const int IR3 = A2;
// motor control pins
const int enablePinA = 5;
const int enablePinB = 6;


// variable definitions
int IR1_val = 0; //reading from IR sensor 1
int IR2_val = 0;
int IR3_val = 0;
int mean_val = 0;
int error = 0;
//int white_mean = 22;
//int black_mean = 33;
int IR1_bias = ;
int IR2_bias = 0;
int IR3_bias = 1;

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
  pinMode(enablePinA, OUTPUT);
  pinMode(enablePinB, OUTPUT);
}

void loop()
{

  // read the IR sensors:
  IR1_val = analogRead(IR1) - IR1_bias; //left looking from behind
  IR2_val = analogRead(IR2) - IR2_bias; //centre
  IR3_val = analogRead(IR3) - IR3_bias; //right

  // process IR readings
  mean_val = (IR1_val + IR2_val + IR3_val) / 3;
  error_last = error; //store previous error before new one is caluclated
  error = IR1_val - IR3_val;

  Kp = 8;
  Ki = 0;
  Kd = 3;

  // calculate proportional term
  P_error = error * Kp;

  // calculate integral term
  I_error = I_error + error;
  if (I_error > I_limit)
    I_error = I_limit;
  if (I_error < (-I_limit))
    I_error = -I_limit;

  // calculate differential term
  D_error = (error_last - error)*Kd;

  correction = P_error + D_error;
  cruiseSpeed = 0;

  // set motor speed
  if (correction < 0)
    speedA = (cruiseSpeed + correction);
  else
    speedA = cruiseSpeed;

  if (correction > 0)
    speedB = (cruiseSpeed - correction);
  else
    speedB = cruiseSpeed;

  motor(speedA, speedB);


  // print values
  if (printdelay == 20) {
    printvalues();
    printdelay = 1;
  }
  else {
    printdelay = printdelay + 1;
  }

  delay(30);

}

// motor controller function
void motor(int speedA, int speedB)
{

  // deadband
  if (speedA < 5)
    speedA = 0;
  if (speedB < 5)
    speedB = 0;
  // upper cutoff
  if (speedA > 255)
    speedA = 255;
  if (speedB > 255)
    speedB = 255;

  analogWrite(enablePinA, (255 - speedA));
  analogWrite(enablePinB, (255 - speedB));

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


