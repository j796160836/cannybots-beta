//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// PID Method 1 
//

#ifdef PID_METHOD_1
#define pid_t int
#define pid_m 1

pid_t Kp = 0;
pid_t Ki = 0;
pid_t Kd = 0;

pid_t P_error = 0;
pid_t D_error = 0;
pid_t error = 0;
pid_t error_last = 0; // to calculate D_error = error - error_last
pid_t correction = 0; //error after PID filter

void setup_PID() {
  getPIDSettings();
}

unsigned long pidLastTime = millis();
void calculate_PID() {
 
  if ( (loopNowTime - pidLastTime) < PID_SAMPLE_TIME) {
    return;
  }
  pidLastTime= loopNowTime;
  
  // process IR readings via PID
  error_last = error;                                   // store previous error before new one is caluclated
  //error = constrain(IRvals[0] - IRvals[2], -30, 30);        // set bounds for error
  error = IRvals[0] - IRvals[2];

  P_error = error * Kp / PID_DIV;                               // calculate proportional term
  D_error = (error - error_last) * Kd / PID_DIV;                // calculate differential term
  correction = P_error + D_error;
  cruiseSpeed = baseCruiseSpeed + manualA;
#ifdef MOTOR_A_POS_IS_FORWARD
  speedA = constrain(cruiseSpeed + correction, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  speedB = constrain(cruiseSpeed - correction, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
#else
  speedA = constrain(cruiseSpeed - correction, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  speedB = constrain(cruiseSpeed + correction, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
#endif

}

void update_PID(int _Kp, int _Ki, int _Kd) {
  setPID_P(_Kp);
  setPID_I(_Ki);
  setPID_D(_Kd);
  cb.nvSetInt(&NV_PID_P, _Kp);
  cb.nvSetInt(&NV_PID_I, _Ki);
  cb.nvSetInt(&NV_PID_D, _Kd);
}

void disable_PID() {
}


void enable_PID() {
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
  CB_DBG2REMOTE(    "%lu(%lu@%lu): IR(%u,%u,%u),Kpd(%d,%d)/100,Sab(%d,%d),Mab(%d,%d),XY(%d,%d),MEM(%d)\n\n", //VCC(%d)", // e(%d) PeDe(%d,%d)
             loopNowTime,
             loopDeltaTime,
             loopcount,
             IRvals[0], IRvals[1], IRvals[2],
             Kp*100, Kd*100, 
             //error, P_error, D_error,
             speedA, speedB, manualA, manualB,
             xAxisValue, yAxisValue,
             cb.getFreeMemory()
             //ANALOG_READ(BATTERY_PIN)
        );
}

#endif



