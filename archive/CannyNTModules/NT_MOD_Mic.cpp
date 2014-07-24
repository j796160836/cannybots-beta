
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Mic
// when id=0 => all sensors,

#ifdef USE_MIC
uint8_t mic_generateReadings(uint8_t id) {
  uint8_t status;
  if (0 == id) {
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD1(NT_CAT_MIC, NT_CMD_MIC_RESULT, 1, analogRead(MIC_PIN)),
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

