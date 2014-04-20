#ifndef NTProtocol_h
#define NTProtocol_h




// protocol is 'fire and forget' asyncronous.  no DACK and no correlated PACK
// The BlueFruit UART service has the RX characteristic defined to be to 'write with no reply'
//

//  20 byte 'wide' commandbuffer (BLE device/stack   restriction/optimisation)
//  fixed 4 bytes per command tuple = up to 4 commands per 'wide' command
//
//  tuple 1:  header
//  tuple 2:  cmd 0
//  tuple 3:  cmd 1
//  tuple 4:  cmd 2
//  tuple 5:  cmd 3

// OR

//  tuple 1:  header
//  tuple 2:  cmd 0        (with cont bit set)
//  tuple 3:  args 2 + 3   (2x 16bit values 
//  tuple 4:  cmd 1
//  tuple 5:  cmd 2

// OR

//  tuple 1:  header
//  tuple 2:  cmd 0                (with cont bit set)
//  tuple 3:  args 2 + 3 for cmd0  (2x 16bit values)
//  tuple 4:  cmd 1                (with cont bit set)
//  tuple 5:  args 2 + 3 for cmd1


//  Header:
//  byte 1 : 0xFF
//  byte 2 : 0x01    version 
//  byte 3 : 0x[TargetID]  uniqie device targer id netween 1-250 (special values: 0 = dontcare, usual master/slave model, 255 = broadcast)
//  byte 4 : 0x[CRC]

// Commmand Tuples:
//  byte 1 :  <bifield>
//            bits 7-5  Category    cmd catagory  (e.g. AX12, sonar, etc)
//            bit  4    Continue
//            bits 3-0  Command      (e.g. category specifc, e.g. setVelPos, ping for Sonar)
//
//  byte 2   : id            implemnetation defined, but typically an attached device id, e.g. AX12 server ID, Sonar sensor number
//  byte 3&4 : p1            command paramter - a 16 bit value 

//
//  Catageories: (0-7)
//                0  =  NOP
//                1  =  AX12 - Actions
//                2  =  AX12 - Sensor readings
//                3  =  Sensor Commands (e.g. sonar:  active/inactive, trigger one shot reading, contiuous ON/OFF)
//                4  =  Sensor Readings (e.g. sonar, IR)
//                5  =  Pixy Commands and Readings
//                6  =  PWM Servo & Motor Commands and Readings
//                7  =  System commands (e.g. used for configuration, status reporting or system control)




//  Continue:     0 = next 4 bytes are un releated.
//                1 = next 4 bytes are used by this command as 2x 16 bit parameters

//  Command:  (0-15):   category specifc (see below)

#define NT_WIRE_PROTOCOL_MAGIC   0xFF
#define NT_WIRE_PROTOCOL_VERSION 0x01
#define NT_MSG_HEADER_BYTES 4
#define NT_MSG_COMMAND_BYTES 4

#include <stdint.h>

typedef struct {
  uint8_t    category;
  uint8_t    command;
  uint8_t    argc;
  uint8_t    id;
  int16_t arg1;
  int16_t arg2;
  int16_t arg3;
  int16_t arg4;
} NT_cmd;

// Function decls.

#if __cplusplus
extern "C" {
#endif
  uint8_t NT_validateMsg(uint8_t *buffer, uint8_t len);
  uint8_t NT_calculateMsgCRC(uint8_t *buffer);
  uint8_t NT_extractCommand(uint8_t* buf, uint8_t cmd, NT_cmd *out);
  void NT_sendError(uint8_t err);

  
  // to be implemented by client.
  uint8_t  NT_scheduleMsg(uint8_t* buffer);
  
#if __cplusplus
}
#endif




#define NT_CAT_FROM_MSG_BYTE(b) ( (b & 0xFF) >> 5) 
#define NT_CMD_FROM_MSG_BYTE(b) (b& 0xF)
#define NT_CONT_FROM_MSG_BYTE(b) (b & (1<<4))


// Client utils
#define NT_CMD_NO_CONT 0
#define NT_CMD_CONT 1

#define bytesFromInt(x)   (uint8_t)(x & 0xff), (uint8_t)((x &0xff00) >>8)

//#define copy_bytes16(x, adr) *(adr) =
#define NT_CREATE_CMD(cat, cmd, cont)   ( (cat & 0x7) << 5 )  +  ( (cont & 1) << 4 ) + ( cmd & 0xF)

#define NT_CREATE_CMD_WITH_ARG1(buf, cat,cmd,id,arg1)  \
      buf[0] = NT_CREATE_CMD(cat, cmd, NT_CMD_NO_CONT); \
      buf[1] = id; \
      buf[2] = (arg1 & 0xFF); \
      buf[3] = ((arg1 & 0xFF00) >> 8);
      


#define NT_CREATE_CMD_NOP 0x00, 0x00, 0x00, 0x00

///


#define NT_STATUS_OK                  0
#define NT_STATUS_WARN                1
#define NT_STATUS_ERROR               2
#define NT_STATUS_FATAL               3


#define NT_ERROR_MSG_NONE             0
#define NT_ERROR_MSG_LEN              1
#define NT_ERROR_MSG_HEADER_NOT_MAGIC 2
#define NT_ERROR_MSG_CRC_FAILED       3
#define NT_ERROR_CAT_INVALID          4
#define NT_ERROR_CMD_INVALID          5
#define NT_ERROR_CMD_PARAMS_INVALID   6

#define NT_MSG_SIZE 20
#define NT_ERROR_BYTE_OFFSET  19

#define NT_IS_VALID_MSG(buf)  ( (0xFF == buf[0]) && (NT_WIRE_PROTOCOL_VERSION == buf[1]))
#define NT_IS_VALID_MSG_LEN(len)  ( len == NT_MSG_SIZE)
#define NT_MSG_SET_ERROR(buf, err)  buf[NT_ERROR_BYTE_OFFSET]=err


#define NT_DEFAULT_BUFFER  {  \
    NT_WIRE_PROTOCOL_MAGIC, NT_WIRE_PROTOCOL_VERSION, 0x00, 0x00,  \
    NT_CREATE_CMD_NOP,\
    NT_CREATE_CMD_NOP,\
    NT_CREATE_CMD_NOP,\
    NT_CREATE_CMD_NOP}


#define NT_DEFAULT_MSG_HEADER(x) NT_WIRE_PROTOCOL_MAGIC, NT_WIRE_PROTOCOL_VERSION, 0x00, 0x00
#define NT_MSG_CALC_CRC(buf)  buf[3]=NT_calculateMsgCRC(buf); // CRC check buffer


extern uint8_t NT_lastError;

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Categories


#define NT_CAT_NOP   0
#define NT_CAT_AX12  1
#define NT_CAT_SONAR 4
#define NT_CAT_IRRECV 4
#define NT_CAT_SYSTEM  7


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// System Commands

#define NT_CMD_SYS_LAST_ERROR    1
#define NT_CMD_SYS_INFO_BANNER   4


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Sensor Commands

#define NT_CMD_SONAR_PING         1



//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Sensor Reading 'Commands'

#define NT_CMD_SONAR_PING_RESULT  2
#define NT_CMD_IRRECV_RESULT      3
         


//////////////////////////////////////////////////////////////////////////////////////////////////
//
// AX12 Commands
#define NT_CMD_AX12_TESTS         0
#define NT_CMD_AX12_PING          1
#define NT_CMD_AX12_RESET         2
#define NT_CMD_AX12_SET_ENDLESS_TURN_MODE   10
#define NT_CMD_AX12_SET_ENDLESS_TURN_SPEED  11
#define NT_CMD_AX12_SET_VELPOS    13
#define NT_CMD_AX12_SET_ID        14
#define NT_CMD_AX12_DISCOVER      15
// 0 getConfig()
// 1 ping(id)
// 2 reset(id)
// 3 setVelocity(byte id, int vel) (0-1500 ?)
// 4 setPosition(byte id, int pos) (0-1500 ?)
// 5 AX12data ReadData(byte id, byte start, byte length)
// 6 int WriteData(byte id, byte start, byte length, byte[] values, bool isReg)
// 7 int action(id)
// 8 readInfo
// 9 writeInfo
// 10 void setEndlessTurnMode(byte id, bool endless)
// 11 void endlessTurnSpeed(byte id, int speed)
// 12 int getPresentPSL(id, int* psl) -> psl[3] = [PRESENT_POSITION , PRESENT_SPEED, PRESENT_LOAD]
// 13 int setPosVel(byte id, int pos, int vel)
// 14 void changeId(byte id)
// 15 void setSRM(byte id, byte srl) srm enum of { RETURN_NONE | RETURN_READ | RETURN_ALL }

// Helpers (setters)
// 20 setTorque(byte id, bool torque)
// 21

// broadcasts:
// 40  syncWrite...
// 41  syncInfo...
// 42  setMultiPosVel



#endif
