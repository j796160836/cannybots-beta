
#include "NTProtocol.h"

uint8_t NT_lastError;

void NT_sendError(uint8_t err) {
  NT_lastError = err;
  uint8_t msg[NT_MSG_SIZE] = {
    NT_DEFAULT_MSG_HEADER(),
    NT_CREATE_CMD(NT_CAT_SYSTEM, NT_CMD_SYS_LAST_ERROR, NT_CMD_NO_CONT), 0, bytesFromInt(NT_lastError),
    NT_CREATE_CMD_NOP,
    NT_CREATE_CMD_NOP,
    NT_CREATE_CMD_NOP
  };
  NT_MSG_CALC_CRC(msg);

  NT_scheduleMsg(msg);
}



uint8_t NT_validateMsg(uint8_t *buffer, uint8_t len) {
  if ( ! NT_IS_VALID_MSG_LEN(len) ) {
    return NT_ERROR_MSG_LEN;
  } else if ( !NT_IS_VALID_MSG(buffer)) {
    return NT_ERROR_MSG_HEADER_NOT_MAGIC;
  } else if (NT_calculateMsgCRC(buffer)) {
    return NT_ERROR_MSG_CRC_FAILED;
  }
  return NT_STATUS_OK;
}

uint8_t NT_calculateMsgCRC(uint8_t *buffer) {
  return 0x00;
}
uint8_t NT_validateMsgCRC(uint8_t *buffer) {
  return 0x00;
}

// TODO: check if IS_CONT commands
uint8_t NT_extractCommand(uint8_t* buf, uint8_t cmd, NT_cmd *out) {
  int offset = NT_MSG_HEADER_BYTES + NT_MSG_COMMAND_BYTES * cmd;
  out->category =  ( *(buf + offset) >> 5 ) & 0x7;
  out->command  =  buf[offset] & 0xF;
  out->id = buf[offset + 1];
  // TODO: check if IS_CONT commandsand adjust according ly
  out->argc = (out->command == NT_CAT_NOP)?0:1;
  out->arg1 = buf[offset + 2]  +  ( buf[offset + 3]  << 8);

  if ( *(buf + offset) & 4 ) {
    out->argc = 3;
    out->arg2 = buf[offset + 4]  +  ( buf[offset + 5]  << 8);
    out->arg3 = buf[offset + 6]  +  ( buf[offset + 7]  << 8);
  }

  return NT_STATUS_OK;
}

