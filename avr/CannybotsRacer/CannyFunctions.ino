//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Remotely called funcs

bool forceManualMode = false;

// TODO: to be moved to LineFollowing lib
// Joystick
int yAxisValue = 0;  // -255..255
int xAxisValue = 0;  // -255..255


void lf_updateMotorSpeeds(int _speedA, int _speedB, int _dummy) {
}

void lf_updateAxis(int xAxis, int yAxis, int _dummy) {  
  xAxisValue = xAxis;  //joy X axis vale  = Direction  -255 to 255
  yAxisValue = yAxis;  //joy y axis vale = Throttle    -255 to 255
    CB_DBG("joy=%d,%d", xAxis,yAxis);

}


void lf_updatePID(int _Kp, int _Ki, int _Kd) {
  CB_DBG("PID=%d,%d,%d", _Kp, _Ki, _Kd);
  setPID_P(_Kp);
  setPID_D(_Kd); 
  cb.setConfigParameterValue(&cfg_pid_p, &_Kp);
  cb.setConfigParameterValue(&cfg_pid_d, &_Kd);
}

void lf_updateBias (int b1, int b2, int b3) {
  CB_DBG("Bias=%d,%d,%d", b1, b2, b3);
  IRbias[0] = b1;
  IRbias[1] = b2;
  IRbias[2] = b3;
  // TODO: change to the generic:  cb.setConfigParameterValue(&NV_IRBIAS_1), no need to specify variable address again
  cb.setConfigParameterValue(&cfg_ir_bias_1, &IRbias[0]);
  cb.setConfigParameterValue(&cfg_ir_bias_2, &IRbias[1]);
  cb.setConfigParameterValue(&cfg_ir_bias_3, &IRbias[2]);
}

void lf_updateLineFollowingMode(int _forceManualMode, int _d1, int _d2) {
  CB_DBG("ForceManual=%d", _forceManualMode);
  forceManualMode = _forceManualMode;
}

void lf_emitConfig(int _d1, int _d2, int _d3) {
  cb.callMethod(&RACER_PID, getPID_P(), getPID_I(), getPID_D());
  cb.callMethod(&RACER_IRBIAS, IRbias[0], IRbias[1], IRbias[2]);
}

void lf_emitIRValues(int v1, int v2, int v3) {
  static unsigned long lastCall = millis();
  if (millis() - lastCall > 200) {
    cb.callMethod(&RACER_IRVALS, v1, v2, v3);
    lastCall = millis();
  }
}


void lf_ping(int v1) {
  //CB_DBG("ping", v1)
}

