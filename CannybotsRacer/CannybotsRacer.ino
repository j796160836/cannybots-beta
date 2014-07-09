#include <Cannybots.h>
#include "CannybotsRacer.h"
Cannybots& cb = Cannybots::getInstance();

#define NUM_MOTORS       2
#define NUM_IR_SENSORS   3
int Kp, Ki, Kd;
int speeds[NUM_MOTORS];
int IRbias[NUM_IR_SENSORS];
int baseCruiseSpeed;
int currentCruiseSpeed;
bool isLineFollowingMode;

void setup() {
  cb.setConfigStorage("CBLF", 64);
  cb.registerHandler(RACER_CRUISESPEED, lf_updateMotorSpeeds); 
  cb.registerHandler(RACER_LINEFOLLOWING_MODE, lf_updateLineFollowingMode);   
  cb.registerHandler(RACER_PID, lf_updatePID);  
  cb.registerHandler(RACER_IRBIAS, lf_updateBias);  
  cb.registerHandler(RACER_JOYAXIS, lf_updateAxis);  
  cb.begin();
}

void loop() {
  cb.update();  
  cb.callMethod(RACER_LINEFOLLOWING_MODE, 1);    
  delay(100);
  //CB_DBG("EndLoop. freemem=(%d)",cb.getFreeMemory());
}

void lf_updateMotorSpeeds(int speedA, int speedB, int dummy) {
  CB_DBG("%d,%d", speedA, speedB)
}

void lf_updateAxis(int xAxis, int yAxis, int dummy) {
  CB_DBG("axis=%d,%d", xAxis, yAxis);
}
void lf_updatePID(int Kp, int Ki, int Kd) {
  CB_DBG("PID=%d,%d,%d", Kp, Ki, Kd);
}

void lf_updateBias (int b1, int b2, int b3) {
  CB_DBG("Bias=%d,%d,%d", b1, b2, b3);
}

void lf_updateLineFollowingMode(int isLFMode, int _d1, int _d2) {
  CB_DBG("LF=%d", isLFMode)
}






