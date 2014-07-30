

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef PID_METHOD_1

int Kp = 0;
int Ki = 0;
int Kd = 0;
int I_limit = 100;
int P_error = 0;
int I_error = 0;
int D_error = 0;
int error = 0;
int error_last = 0; // to calculate D_error = error - error_last
int correction = 0; //error after PID filter

void setup_PID() {
  getPIDSettings();
}

void calculate_PID() {
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
  cb.nvSetInt(NV_PID_P, _Kp);
  cb.nvSetInt(NV_PID_I, _Ki);
  cb.nvSetInt(NV_PID_D, _Kd);
}

void disable_PID() {
}


void enable_PID() {
}

void setPID_P(int v) {
  Kp = v;
}
void setPID_I(int v) {
  Ki = v;
}
void setPID_D(int v) {
  Kd = v;
}
int getPID_P() {
  return Kp;
}
int getPID_I() {
  return Ki;
}
int getPID_D() {
  return Kd;
}

void printvals_PID() {
  CB_DBG(    "%lu: IR(%u,%u,%u) Kpd(%d,%d) e(%d) PeDe(%d,%d) Sab(%d,%d) Mab(%d,%d), XY(%d,%d), MEM(%d), ", //VCC(%d)",
             millis(),
             IRvals[0], IRvals[1], IRvals[2],
             Kp, Kd, error, P_error, D_error,
             speedA, speedB, manualA, manualB,
             xAxisValue, yAxisValue,
             cb.getFreeMemory()
             //ANALOG_READ(BATTERY_PIN)
        );
}

#endif



