
#include "NTProtocol.h"
#include "NTUtils.h"

uint8_t  linefollow_processCommand(uint8_t cmd, uint8_t id, int16_t p1);


uint8_t NT_lastError;

void NT_sendError(uint8_t err) {
  NT_lastError = err;
#ifdef ARDUINO
    Serial.print("NT_sendError:");
    Serial.println(err, DEC);
#endif
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




//////////////////////////////////////////////////////////////////////////////////////////////////
// Command Processing


int  processCategoryCont(int cat, int cmd, int id, int p1, int p2, int p3) {
  return -1;
}



uint8_t  processCategory(int cat, int cmd, int id, int p1) {
  uint8_t status = NT_STATUS_OK;
  switch (cat) {
#ifdef USE_AX12
    case NT_CAT_AX12:
      status = ax12_processCommand(cmd, id, p1);
      break;
#endif     
#ifdef USE_SONAR      
    case NT_CAT_SONAR:
      status = sonar_processCommand(cmd, id, p1);
      break;
#endif      
#ifdef USE_TONE
    case NT_CAT_TONE:
      status = tone_processCommand(cmd, id, p1);
      break;
#endif
#ifdef ARDUINO
    case NT_CAT_APP:  
    //case NT_CAT_APP_LINEFOLLOW:
      status = linefollow_processCommand(cmd, id, p1);
#endif
      break;
    case NT_CAT_NOP:  
      break;
    default:
      //debug(F("Invalid category: %x"), cat);
      status=NT_ERROR_CAT_INVALID;
  }
  return status;
}




void processMessage(uint8_t *buffer, uint8_t len)
{
  uint8_t status = NT_validateMsg(buffer, len);

  if ( NT_STATUS_OK == status ) {

    // loop thru the commands
    for (int p = NT_MSG_HEADER_BYTES; p < NT_MSG_SIZE; p += NT_MSG_COMMAND_BYTES) {
      int  cat  = NT_CAT_FROM_MSG_BYTE(buffer[p]);
      int  cmd  = NT_CMD_FROM_MSG_BYTE(buffer[p]);
      
      if (NT_CAT_NOP == cat) 
        continue;
        
      if (! NT_CONT_FROM_MSG_BYTE(buffer[p])) {
        status = processCategory(cat, cmd, buffer[p + 1], mk16bit(buffer[p + 2], buffer[p + 3]));
        if (NT_STATUS_OK != status) {
#ifdef ARDUINO
            Serial.println("ERR1");
#endif
          NT_sendError(status);
        }      
      } else {
        status = processCategoryCont(cat, cmd, buffer[p + 1],
                            mk16bit(buffer[p + 2], buffer[p + 3]),
                            mk16bit(buffer[p + 4], buffer[p + 5]),
                            mk16bit(buffer[p + 6], buffer[p + 7]));
        if (NT_STATUS_OK != status) {
#ifdef ARDUINO
            Serial.println("ERR3");
#endif
          NT_sendError(status);
        }
        p += NT_MSG_COMMAND_BYTES;
      }
    }
  } else {
#ifdef ARDUINO
      Serial.println("ERR3");
#endif

    NT_sendError(status);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// System Commands

uint8_t  system_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  switch (cmd) {
    case NT_CMD_SYS_INFO_BANNER:
    /*
      DBG(("Brain is alive!"));
      DBG(("Voltage: %d mv") << voltage);
      DBG(("Detected: ") << ax12_num);
      DBG(("ping 1: ") << ax12_motors[0].ping() );
      DBG(("ping 2: ") << ax12_motors[1].ping() );
    */
      status = NT_STATUS_OK;
      break;
    default:
      //DBG("system:NT_ERROR_CMD_INVALID!");
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}




