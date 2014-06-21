

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// IR Receiver

#ifdef USE_IRRECV

// this library was moifed, name change to resolve conflit with standard library (which had classh with other lib..)
// modief alto to use TIMER1 and not timer 2 in IRRemoteIntORG.h
#include <IRremoteORG.h>

IRrecv irrecv(IRRECV_RECV_PIN); 

void irrecv_init() {
  irrecv.enableIRIn();
}


void irrecv_dump(decode_results *results);
  decode_results irrecv_results;
  uint8_t irrecv_generateReadings(uint8_t id)  {     // 0 =all,  1 = 1st, 2 = 2nd etc
  uint8_t status = NT_STATUS_OK;

  
  if (irrecv.decode(&irrecv_results)) {
    //int count = irrecv_results.rawlen;
    //irrecv_results.decode_type;    // manufacturrer code define'd constant (e,g,SONY, PANASONIC, RC5, RC6, et.c. see header/example)
    //3irrecv_results.value;          // key press (
    //irrecv_results.panasonicAddress
    
    debug("v=%x t=%x\n", irrecv_results.value, irrecv_results.decode_type, irrecv_results.panasonicAddress);
    irrecv.resume(); // Receive the next value
  
    uint8_t msg[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      //TODO: use the cont and send all three values
      NT_CREATE_CMD(NT_CAT_IRRECV, NT_CMD_IRRECV_RESULT, NT_CMD_NO_CONT), 1, bytesFromInt(irrecv_results.value),
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



