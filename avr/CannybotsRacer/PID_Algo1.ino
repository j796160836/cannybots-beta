//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// PID Method 1 
//

#ifdef PID_METHOD_1


void setup_PID() {
}

unsigned long pidLastTime = millis();
void calculate_PID() {
 
  if ((loopNowTime - pidLastTime) < PID_SAMPLE_TIME){
    return;
  }
  pidLastTime= loopNowTime;

  // process IR readings via PID
  error_last = error;                                   // store previous error before new one is caluclated
  error = IRvals[0] - IRvals[2];

  P_error = error * Kp / PID_DIV;                               // calculate proportional term
  D_error = (error - error_last) * Kd / PID_DIV;                // calculate differential term
  correction = P_error + D_error;

  speedA = constrain(cruiseSpeed + ( MOTOR_A_POS_IS_FORWARD?correction:-correction), -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  speedB = constrain(cruiseSpeed + ( MOTOR_B_POS_IS_FORWARD?correction:-correction), -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

}


void setPID_P(int v) {
  Kp = v/pid_m;
}
void setPID_I(int v) {
  Ki = v/pid_m;
}
void setPID_D(int v) {
  Kd = v/pid_m;
}
int getPID_P() {
  return Kp*pid_m;
}
int getPID_I() {
  return Ki*pid_m;
}
int getPID_D() {
  return Kd*pid_m;
}

void printvals_PID() {
  CB_DBG(    "%lu(%lu@%lu): IR(%u,%u,%u),Kpd(%d,%d)/100,Sab(%d,%d), XY(%d,%d),MEM(%d)\n\n", //VCC(%d)", // e(%d) PeDe(%d,%d)
             loopNowTime,
             loopDeltaTime,
             loopcount,
             IRvals[0], IRvals[1], IRvals[2],
             Kp*100, Kd*100, 
             //error, P_error, D_error,
             speedA, speedB,
             xAxisValue, yAxisValue,
             cb.getFreeMemory()
             //ANALOG_READ(BATTERY_PIN)
        );
}

#endif



