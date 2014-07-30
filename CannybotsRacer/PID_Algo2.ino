//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////// PID Method 2:  PID Library
#include <PID_v1.h>

#ifdef PID_METHOD_2


double Setpoint, Input, Output;
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup_PID() {
  getPIDSettings();
  read_ir_sensors();
  Setpoint = PID_SETPOINT;
  myPID.SetSampleTime(PID_SAMPLE_TIME);
  myPID.SetOutputLimits(- (MOTOR_MAX_SPEED - baseCruiseSpeed), MOTOR_MAX_SPEED - baseCruiseSpeed);
  myPID.SetMode(AUTOMATIC);
}

void calculate_PID() {

  /*
  double sum = IRvals[0] + IRvals[1] + IRvals[2];
  if (sum!=0) {
    Input =  ( (IRvals[2] - IRvals[0])  / sum  ) * 100.0;
  } else {
    Input = 0; 
  }*/
  Input = (IRvals[2] - IRvals[0])/100.0 ;
  
  myPID.Compute();

  cruiseSpeed = baseCruiseSpeed + manualA;

#ifdef MOTOR_A_POS_IS_FORWARD
  speedA = cruiseSpeed + Output;
  speedB = cruiseSpeed - Output;
#else
  speedA = cruiseSpeed - Output;
  speedB = cruiseSpeed + Output;
#endif


}

void update_PID(int _Kp, int _Ki, int _Kd) {
  setPID_P(_Kp);
  setPID_I(_Ki);
  setPID_D(_Kd);
}

void setPID_P(int v) {
  Kp = ((double)v) / 100.0;
  myPID.SetTunings(Kp, Ki, Kd);
  cb.nvSetInt(NV_PID_P, v);
}

void setPID_I(int v) {
  Ki = ((double)v) / 100.0;
  myPID.SetTunings(Kp, Ki, Kd);
  cb.nvSetInt(NV_PID_I, v);
}
void setPID_D(int v) {
  Kd = ((double)v) / 100.0;
  myPID.SetTunings(Kp, Ki, Kd);
  cb.nvSetInt(NV_PID_D, v);
}

int getPID_P() {
  return (int) (myPID.GetKp() * 100.0);
}
int getPID_I() {
  return (int) (myPID.GetKi() * 100.0);
}
int getPID_D() {
  return (int) (myPID.GetKd() * 100.0);
}

void disable_PID() {
  myPID.SetMode(MANUAL);
}

void enable_PID() {
  myPID.SetMode(AUTOMATIC);
}

void printvals_PID() {
  CB_DBG(    "%lu (%lu): IR(%u,%u,%u), IRonB(%d,%d,%d), Kpid(%d,%d,%d)/100, InOut(%d, %d)/100, Sab(%d,%d) Mab(%d,%d), XY(%d,%d), MEM(%d), ", //VCC(%d)",
             millis(),
             loopDeltaTime,
             IRvals[0], IRvals[1], IRvals[2],
             IRonBlack[0], IRonBlack[1], IRonBlack[2],
             (int) (myPID.GetKp() * 100), (int)(myPID.GetKi() * 100), (int)(myPID.GetKd() * 100),
             (int) (Input * 100), (int)(Output * 100),
             speedA, speedB, manualA, manualB,
             xAxisValue, yAxisValue,
             cb.getFreeMemory()
             //ANALOG_READ(BATTERY_PIN)
        );
}

#endif
