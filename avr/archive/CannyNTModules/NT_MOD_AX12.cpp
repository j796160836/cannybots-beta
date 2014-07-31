
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// AX12 


#ifdef USE_AX12
#include <ax12.h>

// Missing in 3rd party library:
#define ERR_NONE                    0
#define ERR_VOLTAGE                 1
#define ERR_ANGLE_LIMIT             2
#define ERR_OVERHEATING             4
#define ERR_RANGE                   8
#define ERR_CHECKSUM                16
#define ERR_OVERLOAD                32
#define ERR_INSTRUCTION             64


byte ax12_detect[AX12_MAX_SERVOS];
AX12 ax12_motors[AX12_MAX_SERVOS];
byte ax12_num;



void ax12_setup() {
  
  AX12::init (1000000);   // AX12 at 1 Mb/s
  ax12_autodetect();
  ax12_defaults();
}

void ax12_defaults() {
  
  for (int i =0; i< ax12_num; i++) {
    if (i<ax12_num) {
      int mid =  ax12_detect[i];
      ax12_motors[i].id=mid;
      ax12_motors[i].setEndlessTurnMode(true);
      ax12_motors[i].endlessTurn(0);
    }
  }
  if (ax12_num>=2)
    ax12_motors[1].inverse=true;   
}

uint8_t ax12_autodetect() {
   ax12_num = AX12::autoDetect (ax12_detect, AX12_MAX_SERVOS);
   debug(F("num=%d"), ax12_num);
}


uint8_t  ax12_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  

  AX12* motor = &ax12_motors[id-1];
  
  switch (cmd) {
    case NT_CMD_AX12_PING:
      status = motor->ping()==0?NT_STATUS_OK:NT_ERROR_CMD_GENERIC_ERROR;
      break;
    case NT_CMD_AX12_RESET:
      motor->reset();
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ENDLESS_TURN_MODE:
      motor->setEndlessTurnMode(p1>0?true:false);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ENDLESS_TURN_SPEED:
      motor->endlessTurn(p1);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_POS:
      motor->setPos(p1);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_VEL:
      motor->setVel(p1);
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_VELPOS:
      // TODO: move to multi arg command..
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_SET_ID:
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_DISCOVER:
      ax12_autodetect();
      status = NT_STATUS_OK;
      break;
    case NT_CMD_AX12_TESTS:
      status = ax12_runTests(id, p1);
      break;
    //case NT_CMD_SCAN_RESET:
    //   for (int i=1;  i< 254; i++) {
    //       AX12 m = AX12(id)
    //       m->changeID(id);
    //   }
    
    default:
      status = NT_ERROR_CMD_INVALID;
  }
  
  return status;
}

uint8_t  ax12_runTests(int id, int p1) {
  //AX12 motor = ax12_motors[0];
  ax12_motors[0].setSRL(RETURN_ALL);
  AX12info result;
  result = ax12_motors[0].readInfo (RETURN_DELAY_TIME);
  byte val = result.value;
  int  err = result.error;
  debug("val=%x,err=%x", val, err);
  debug("ptr=%x,ax[0]=%x", AX12::ax_rx_Pointer, AX12::ax_rx_buffer[0]);


  delay(15);
  // ping all +1 for an error ping value
  for (byte i = 0 ; i< 5; i++) {
    AX12 m = AX12(i+1);
    debug("p(%d,%x)=%x", i,  m.id,  m.ping());
  }


  ax12_defaults();
}
#endif

