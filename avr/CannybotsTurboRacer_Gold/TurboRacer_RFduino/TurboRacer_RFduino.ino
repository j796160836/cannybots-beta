//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots LineFollowing Robot - RFduino
//
// Authors:  Wayne Keenan & Anish Mampetta
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0  -  22.09.2014  -  Inital Version  (Wayne Keenan & Anish Mampetta)
//
//////////////////////////////////////////////////////////////////////////////////////////////////

void joypadManualControlMode() {

  // If the xis readings are small set them to 0
  if ( abs(xAxisValue) < JOYPAD_AXIS_DEADZONE)
    xAxisValue = 0;
  if ( abs(yAxisValue) < JOYPAD_AXIS_DEADZONE)
    yAxisValue = 0;

  speedA =  (yAxisValue + (xAxisValue / 2)) / 3.0;
  speedB =  (yAxisValue - (xAxisValue / 2)) / 3.0;
  //speedA =  xAxisValue;
  //speedB =  -yAxisValue;

}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Inputs

// Sensors
void readIRSensors() {
  IRvals[0] = analogRead(IR1_PIN) + IR1_BIAS; //left looking from behind
  IRvals[1] = analogRead(IR2_PIN) + IR2_BIAS; //centre
  IRvals[2] = analogRead(IR3_PIN) + IR3_BIAS; //right
}

// Joypad

// this is called when a data packet is received via the radio, do very little processing, just set some vars.
// NOTE: because this is called as a background event then any printed value may be more recent than the value used in the prior PID calc.
void joypad_update(int x, int y, int b) {
  xAxisValue = x;
  yAxisValue = -y;
  buttonPressed = b;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Outputs

void motorSpeed(int _speedA, int _speedB) {
  _speedA = constrain(_speedA, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  _speedB = constrain(_speedB, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

  digitalWrite(MOTOR_A1_PIN, _speedA >= 0 ? HIGH : LOW) ;
  analogWrite (MOTOR_A2_PIN, abs(_speedA));

  digitalWrite(MOTOR_B1_PIN, _speedB >= 0 ? HIGH : LOW);
  analogWrite (MOTOR_B2_PIN, abs(_speedB));
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

void updateLineFollowingStatus() {
  if ((IRvals[1] <= IR_WHITE_THRESHOLD )) {
    offTheLineTime += timeNow - offLineLastTime;
    offLineLastTime = timeNow;

    if (offTheLineTime > OFF_LINE_MAX_TIME) {
      isLineFollowingMode = 0;
    }
  } else {
    offTheLineTime = 0;
    isLineFollowingMode = 1;
  }

  if (buttonPressed)
    forceManualMode = 1;
  else
    forceManualMode = 0;

  if (forceManualMode) {
    isLineFollowingMode = 0;
  }
}

void printVals() {
  static unsigned long lastPrint = millis();
  if (millis() - lastPrint < 200) {
    return;
  }
  lastPrint = millis();

  Serial.print(timeNow);
  Serial.print(":IR=(");
  Serial.print(IRvals[0], DEC);
  Serial.print(",");
  Serial.print(IRvals[1], DEC);
  Serial.print(",");
  Serial.print(IRvals[2], DEC);
  Serial.print("),errors =(");
  Serial.print(P_error, DEC);
  Serial.print(",");
  Serial.print(D_error, DEC);
  Serial.print(",");
  Serial.print(I_error, DEC);
  Serial.print(",");
  Serial.print("),Speed(A,B)=(");
  Serial.print(speedA, DEC);
  Serial.print(",");
  Serial.print(speedB, DEC);
  Serial.print("),Joy(X,Y,Button)=(");
  Serial.print(xAxisValue, DEC);
  Serial.print(",");
  Serial.print(yAxisValue, DEC);
  Serial.print(",");
  Serial.print(buttonPressed, DEC);
  Serial.println(")");
}

#ifdef DEBUG
void dumpAnalogReadingsForAllPins() {
  while (1) {
    for (int pin = 2; pin < 7; pin++) {
      analogRead(pin);
      Serial.print(pin, DEC);
      Serial.print("=");
      Serial.print( analogRead(pin), DEC);
      Serial.print("\t");
    }
    Serial.println("\t");
  }
}

#endif
void motorTest() {
  /*motorSpeed(128, 128);   delay(500);
  motorSpeed(-128, -128);   delay(500);
  motorSpeed(128, -128);   delay(500);
  motorSpeed(-128, 128);   delay(500);
  motorSpeed(0, 0);     delay(250);
  */
  
  motorSpeed(128, 0);   delay(500);
  motorSpeed(0, 0);     delay(250);
  motorSpeed(-128, 0);   delay(500);
  motorSpeed(0, 0);     delay(250);
  motorSpeed(0, 128);   delay(500);
  motorSpeed(0, 0);     delay(250);
  motorSpeed(0, -128);  delay(500);
  motorSpeed(0, 0);
}

