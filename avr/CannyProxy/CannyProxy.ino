#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>
#include <stdint.h>
#include <Cannybots.h>


#define MAJOR_VERSION 1
#define MINOR_VERSION 0

//#define USE_SPI 1
#define USE_SPI_PROGRAMMER 1

#define TOGGLE_MILLIS 2000
#define GZLL_TIMEOUT 5000

#if  defined(USE_SPI) || defined(USE_SPI_PROGRAMMER)
#include <SPI.h>
#endif

#define WATCHDOG_SETUP(seconds) NRF_WDT->CRV = 32768 * seconds; NRF_WDT->TASKS_START = 1;
#define WATCHDOG_RELOAD() NRF_WDT->RR[0] = WDT_RR_RR_Reload;

/////////////////////////////////////////////////////////
// Config

#define BLE_UUID                 "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_LOCALNAME            "Cannybots"
#define BLE_ADVERTISEMENT_DATA   "CB_LFB_001"
// Note: BLE_ADVERTISEMENT_DATA must be < 16 bytes.

#define BLE_TX_POWER_LEVEL  -20
#define GZLL_TX_POWER_LEVEL -20
// x one of;  (low) -20, -16, -12, -8, -4, 0, +4 (high & default)
// RFduino default is +4


// TODO: append devideID to advertising data


#define Q_MAX_ENTRIES 12
#define Q_ENTRY_SIZE 20

#define UART_SOURCE Serial
#define SERIAL_BUF_SIZE 32


#ifdef USE_SPI
#define TX_PIN 1
#define RX_PIN 0
#else
#define TX_PIN 5
#define RX_PIN 6
// Gold board
//#define TX_PIN 1
//#define RX_PIN 0
#endif


//TODO: incorporate proxy debug into cannybots lib
static char _proxy_dbg_buffer[20] = {'>', '>'};
#ifdef USE_SPI
#define DBG(FMT, ...) snprintf(_proxy_dbg_buffer+2, 18, FMT, __VA_ARGS__); Serial.write((uint8_t*)_proxy_dbg_buffer,20);Serial.println("");Serial.flush();
#else
#define DBG(FMT, ...) snprintf(_proxy_dbg_buffer+2, 18, FMT, __VA_ARGS__); writeUART((uint8_t*)_proxy_dbg_buffer,20);
#endif // USE_SPI


volatile uint8_t sendClientConnect  = 0;     // 0=false, 1 = BLE, 2 =GZLL
volatile bool sendClientDisconnect = false;




/////////////////////////////////////////////////////////
// Queues

char radio2uartQ[Q_MAX_ENTRIES][Q_ENTRY_SIZE];
int  radio2uartQHead  = 0;
int  radio2uartQTail  = 0;
int  radio2uartQItems = 0;



void enqueue(char *data, int len) {
  if (radio2uartQItems == Q_MAX_ENTRIES)
    return;
  memcpy(radio2uartQ[radio2uartQTail++], data, len <= Q_ENTRY_SIZE ? len : Q_ENTRY_SIZE);
  radio2uartQItems++;
  if (radio2uartQTail == Q_MAX_ENTRIES) {
    radio2uartQTail = 0;
  }
}

/////////////////////////////////////////////////////////
// Serial

//TODO: with 'role' state the Cannybots lib it could support both sides of this BLE proxy (see: Cannybots::sendMessage(Message*))
void sendMessage2localDevice(Message* msg) {
  writeUART(msg->payload, msg->size);
}

void writeUART(const uint8_t* data, uint16_t len) {

#ifdef USE_SPI
  digitalWrite(SS, LOW);    // SS is pin 10
  SPI.transfer ('>');
  SPI.transfer ('>');
  for (int i = 0 ; i < len ; i++) {
    DBG("SPI:%x", data[i]);

    SPI.transfer (data[i]);
  }
  digitalWrite(SS, HIGH);

#else
  Serial.write(">>");
  Serial.write( data, len);
  Serial.flush();              // crashes prior to rfduino 2.0.3
#endif //USE_SPI  
}

void process_uart2ble_q() {
  static uint8_t serialBuffer[SERIAL_BUF_SIZE + 1];
  static int serialBufPtr = 0;
  static bool foundStart = false;
  static char c = 0, lastChar = 0;

  while (UART_SOURCE.available() > 0) {
    lastChar = c;
    c =  UART_SOURCE.read();
    if (foundStart && (serialBufPtr < SERIAL_BUF_SIZE)) {
      serialBuffer[serialBufPtr++] = c;
    } else if ( ('>' == c) && ('>' == lastChar) ) {
      foundStart = true;
      serialBufPtr = 0;
    }
    if (serialBufPtr >= CB_MAX_MSG_SIZE) {
      serialBufPtr = 0;
      foundStart = false;
      if ( ! processMessage(serialBuffer, CB_MAX_MSG_SIZE) ) {
        RFduinoBLE.send((const char*)serialBuffer, CB_MAX_MSG_SIZE);
      }
    }
  }
}

// handle messages that are target at this proxy, return true if processed so that downstream code will not foward to client.
bool processMessage(const uint8_t* buf, uint16_t len) {
  bool messageProcessed = false;
  // TODO: maybe make use of cannybots lib (DRY)
  if (  ( buf[0] == 'C' ) && ( buf[1] == 'B' ) && ( buf[CB_MSG_OFFSET_CMD] == _CBID_CMD_SYS_CALL ) ) {

    // first 2 bytes of data payload is U16 syscall sub-command
    uint8_t cmd = buf[CB_MSG_OFFSET_DATA + 1];
    uint16_t id = 0;
    uint8_t  verMaj = 0, verMin = 0;
    static char advString[BLE_ADVERTISEMENT_DATA_MAX];
    static char nameString[BLE_LOCALNAME_MAX];

    switch (cmd) {

      case _CB_SYSCALL_BLEPROXY_STARTUP:
        DBG("RCVSTART", cmd);
        messageProcessed = true;
        break;

      case _CB_SYSCALL_BLEPROXY_SET_ID_VER:
        id     = (buf[CB_MSG_OFFSET_DATA + 2] << 8) + buf[CB_MSG_OFFSET_DATA + 3];
        verMaj = buf[CB_MSG_OFFSET_DATA + 4];
        verMin = buf[CB_MSG_OFFSET_DATA + 5];

        snprintf(advString, BLE_ADVERTISEMENT_DATA_MAX, "ID(%x)", id);
        snprintf(nameString, BLE_LOCALNAME_MAX, "Cannybot[%x][%d.%d]", id, verMaj, verMin);

        DBG("RCVSETVI=%x,%d.%d", id, verMin, verMaj);
        DBG("A=%s", advString);
        DBG("N=%s", nameString);
        RFduinoBLE.advertisementData = advString;
        RFduinoBLE.deviceName = nameString;

        messageProcessed = true;
        break;

      case _CB_SYSCALL_BLEPROXY_PING:
        DBG("RCVPING", cmd);
        messageProcessed = true;
        break;
      case _CB_SYSCALL_BLEPROXY_SET_TYPE:
      case _CB_SYSCALL_BLEPROXY_SET_AUTH:
      case _CB_SYSCALL_BLEPROXY_SET_TX_PWR:
      case _CB_SYSCALL_BLEPROXY_SLEEP:
      default:
        DBG("?SYSCALL:%d", cmd);
        messageProcessed = true;
        break;

    }
  }

  return messageProcessed;
}

/////////////////////////////////////////////////////////
// Radio

boolean volatile ble_connected = false;
boolean volatile gzll_connected = false;

// GZLL
volatile bool ISPConnected = false;

volatile static unsigned long GZLLPacketTimeout = 0;

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  gzll_connected = true;
  GZLLPacketTimeout = millis() + GZLL_TIMEOUT;
  if (DEVICE1 == device ) {
    ISPConnected = true;
  }

  if (ISPConnected ) {
    rfd_isp_RFduinoGZLL_onReceive(device, rssi, data, len);
  } else {
    enqueue(data, len);
  }
  // TODO: fake pairing. check the device is the same as before...
  // MAYBE-TODO: check uart2ble here as Gazelle can only piggy back client device requests
  //RFduinoGZLL.sendToDevice(device, "OK");
}


// BLE
void RFduinoBLE_onAdvertisement(bool start) {
  //DBG("RFd_BLE_A=%d", start);
}

void RFduinoBLE_onConnect() {
  ble_connected = true;
  sendClientConnect = 1;
  //DBG("RFd_BLE_CON", 0);
}

void RFduinoBLE_onDisconnect() {
  ble_connected = false;
  sendClientDisconnect = true;
  //DBG("RFd_BLE_DIS", 0);
}

void RFduinoBLE_onReceive(char *data, int len) {
  enqueue(data, len);
}


// BLE & GZLL

volatile byte   state  = 0;
volatile unsigned long lastRadioToggleTime = millis();

void  ble_rfduino_manageRadios() {
  unsigned long timeNow   = millis();
  //DBG("time=%ld", timeNow);

  // have to use ABS because of the asyncronous delivery of radio packets and unsigned longs
  // e.g. the GZLL packet may arrive 'sooner' that the value of 'timeNow'
  if ( timeNow  > GZLLPacketTimeout ) {
    if (gzll_connected) {
      sendClientDisconnect = true;
    }
    gzll_connected = ISPConnected = false;

    //DBG("GZTN=%ld", timeNow);
    //DBG("GZTO=%ld", GZLLPacketTimeout);
  }

  if (ble_connected) {
    return;
  }
  if (gzll_connected) {
    return;
  }
  //DBG("SBG=(%d,%d,%d)", state, ble_connected, gzll_connected);

  // move to a next state based on a timeout
  if ( (timeNow - lastRadioToggleTime) > TOGGLE_MILLIS) {
    lastRadioToggleTime =  timeNow;
    state++;
    //DBG("NewState=%d", state);
  }

  // Simple FSM:
  if ( 0 == state) {
    DBG("RFd_BLE_ON", 0);
    RFduinoBLE.begin();
    state++;
  } else if (2 == state  ) {
    DBG("RFd_BLE_OFF", 0);
    while (RFduinoBLE.radioActive);
    RFduinoBLE.end();
    state++;
  } else if (4 == state) {
    DBG("RFd_GZL_ON", 0);
    while (RFduinoBLE.radioActive);
    RFduinoGZLL.begin(HOST);
    state++;
  } else if (6 == state) {
    DBG("RFd_GZL_OFF", 0);
    RFduinoGZLL.end();
    state++;
  } else if (state >= 8) {
    state = 0;
  }

}


/////////////////////////////////////////////////////////
// Queue processing

void process_ble2uart_q() {
  if (radio2uartQItems > 0) {
    writeUART((uint8_t*) radio2uartQ[radio2uartQHead++], Q_ENTRY_SIZE);
    radio2uartQItems--;
    if (radio2uartQHead == Q_MAX_ENTRIES) {
      radio2uartQHead = 0;
    }
  }
}


/////////////////////////////////////////////////////////
// Syscalls


void sendSyscall_Startup() {
  Message* msg = new Message();
  Cannybots::createMessage(msg, &_CB_SYS_CALL, _CB_SYSCALL_BLEPROXY_STARTUP, MAJOR_VERSION, MINOR_VERSION);
  sendMessage2localDevice(msg);
  delete msg;
}

void sendSyscall_Ping() {
  Message* msg = new Message();
  Cannybots::createMessage(msg, &_CB_SYS_CALL, _CB_SYSCALL_BLEPROXY_PING);
  sendMessage2localDevice(msg);
  delete msg;
}


void sendSyscall_ClientConnect(int type) {
  if (!sendClientConnect)
    return;
  sendClientConnect = 0;
  Message* msg = new Message();
  Cannybots::createMessage(msg, &_CB_SYS_CALL, _CB_SYSCALL_BLEPROXY_CLIENT_CONNECT, type);
  sendMessage2localDevice(msg);
  delete msg;
}

void sendSyscall_ClientDisconnect() {
  if (!sendClientDisconnect)
    return;
  sendClientDisconnect = 0;
  Message* msg = new Message();
  Cannybots::createMessage(msg, &_CB_SYS_CALL, _CB_SYSCALL_BLEPROXY_CLIENT_DISCONNECT);
  sendMessage2localDevice(msg);
  delete msg;
}



//TODO: notify a* of client connect & disconencts, e.g. use sendSyscall_ClientConnect and sendSyscall_ClientDisonnect above in the /main loop/ (+ones hot guards)




/////////////////////////////////////////////////////////
// Standard arduino entry points.

void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);        // UART Baud is limited to 9600 when the BLE stack is on.

#ifdef USE_SPI

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);  // ensure SS stays high for now


  SPI.begin ();

  // Slow down the master a bit
  SPI.setFrequency(125);
  // Below is 'standard Arduino' but is actually a no-op on RFduino SDK v2.0.3
  //SPI.setClockDivider(SPI_CLOCK_DIV8);

#endif // USE_SPI

  rfd_isp_setup();
  WATCHDOG_SETUP(7);
  DBG("RFd_PRXSTART!", 0);

  RFduinoBLE.customUUID = BLE_UUID;
  RFduinoBLE.deviceName = BLE_LOCALNAME;
  RFduinoBLE.advertisementData = BLE_ADVERTISEMENT_DATA;
  //RFduinoBLE.advertisementInterval(millis);

  RFduinoBLE.txPowerLevel  = BLE_TX_POWER_LEVEL;
  RFduinoGZLL.txPowerLevel = GZLL_TX_POWER_LEVEL;

  sendSyscall_Startup();
}

void loop() {
  WATCHDOG_RELOAD();
  //DBG("b,g,I=(%d,%d,%d)",ble_connected, gzll_connected, ISPConnected);

  ble_rfduino_manageRadios();

  if (ISPConnected) {
    rfd_isp_loop();
    return;
    //TODO: set ISPConnected after a inactivity delay (or 'bootup' message from A* on Serial/SPI)
  }
  process_ble2uart_q();
  process_uart2ble_q();

  if (sendClientConnect)
    sendSyscall_ClientConnect(sendClientConnect);

  if (sendClientDisconnect)
    sendSyscall_ClientDisconnect();

}
