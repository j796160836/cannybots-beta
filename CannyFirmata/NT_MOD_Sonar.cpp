
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Sonar 



#ifdef USE_SONAR
#include <NewPing.h>

// This library wwas modified to stop the usage of TIMER@ which conflist wijt the IRremote library
// see:https://code.google.com/p/arduino-new-ping/wiki/HELP_Error_Vector_7_When_Compiling
//



NewPing sonars[2] = {NewPing (TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE), NewPing(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE)};


uint8_t  sonar_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  switch (cmd) {
    case NT_CMD_SONAR_PING:
      status = sonar_generateReadings(id);
      break;
    default:
      DBG("sonar:NT_ERROR_CMD_INVALID!");
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}

uint8_t sonar_ping(uint8_t id) {
  unsigned int uS = sonars[ id == 1 ? 0 : 1].ping();
  return uS / US_ROUNDTRIP_CM;   // distance in whole CM's
}

// when id=0 => all sensors,
uint8_t sonar_generateReadings(uint8_t id) {
  uint8_t status;
  if (0 == id) {
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD1(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, 1, sonar_ping(1)),
      NT_CREATE_CMD1(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, 2, sonar_ping(2)),
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    status = NT_scheduleMsg(msg);

  } else {
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD(NT_CAT_SONAR, NT_CMD_SONAR_PING_RESULT, NT_CMD_NO_CONT), id, bytesFromInt(sonar_ping(id)),
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    status = NT_scheduleMsg(msg);

  }
  return status;
}

#endif



