#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>
#include <stdint.h>

//#define USE_SPI 1
#ifdef USE_SPI
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
// x one of;  (low) -20, -16, -12, -8, -4, 0, +4 (high)
// RFduino default is +4


// TODO: append devideID to advertising data

#define TOGGLE_MILLIS 2500

#define Q_MAX_ENTRIES 12
#define Q_ENTRY_SIZE 20

#define UART_SOURCE Serial
#define SERIAL_BUF_SIZE 32

#define NT_MSG_SIZE 20

#ifdef USE_SPI
#define TX_PIN 1
#define RX_PIN 0
#else
//#define TX_PIN 5
//#define RX_PIN 6
#define TX_PIN 1
#define RX_PIN 0
#endif


static char dbg_buffer[20] = {'>', '>'};
#ifdef USE_SPI
#define DBG(FMT, ...) snprintf(dbg_buffer+2, 18, FMT, __VA_ARGS__); Serial.write((uint8_t*)dbg_buffer,20);Serial.println("");Serial.flush();
#else
#define DBG(FMT, ...) snprintf(dbg_buffer+2, 18, FMT, __VA_ARGS__); writeUART((uint8_t*)dbg_buffer,20);
#endif // USE_SPI



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
      radio2uartQTail=0;
    }
}

/////////////////////////////////////////////////////////
// Serial

uint8_t serialBuffer[SERIAL_BUF_SIZE+1];
int serialBufPtr = 0;
bool foundStart = false;
char c=0, lastChar=0;


void writeUART(const uint8_t* data, uint16_t len) {
#ifdef USE_SPI
   SPI.transfer ('>');
   SPI.transfer ('>');
   for (int i = 0 ; i <len ; i++) {
           DBG("SPI:%x",data[i]);

        SPI.transfer (data[i]);
   }
   
#else
  Serial.write(">>");
  Serial.write( data, len);
  Serial.flush();              // crashes prior to rfduino 2.0.3
#endif //USE_SPI  
}

void process_uart2ble_q() {
  while (UART_SOURCE.available()>0) {
        lastChar = c;
        c =  UART_SOURCE.read();
        if (foundStart && (serialBufPtr<SERIAL_BUF_SIZE)) {
            serialBuffer[serialBufPtr++] = c;
        } else if ( ('>' == c) && ('>' == lastChar) ) {
            foundStart=true;
            serialBufPtr=0;
        }
        if (serialBufPtr>=NT_MSG_SIZE) {
            foundStart = false;
            RFduinoBLE.send((const char*)serialBuffer, NT_MSG_SIZE);
            serialBufPtr=0;
        }
    }
}

/////////////////////////////////////////////////////////
// Radio

boolean volatile ble_connected = false;
boolean volatile gzll_connected = false;



// GZLL
static unsigned long lastGZLLPacketTime = millis();
void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  gzll_connected = true;
  lastGZLLPacketTime = millis();
  enqueue(data, len);
  // TODO: fake pairing. check the device is the same as before...
  // TODO: timout connection and if no packet is receve > TIMOUT then go back to BLE/GZll flipping
  // MAYBE-TODO: check uart2ble here as Gazelle can only piggy back client device requests
  //RFduinoGZLL.sendToDevice(device, "OK");
}


// BLE
void RFduinoBLE_onAdvertisement(bool start) {
    DBG("RFd_BLE_A=%d",start);
}

void RFduinoBLE_onConnect() {
  ble_connected = true;
    DBG("RFd_BLE_CON",0);
}

void RFduinoBLE_onDisconnect() {
  ble_connected = false;
  DBG("RFd_BLE_DIS",0);
}

void RFduinoBLE_onReceive(char *data, int len) {
  enqueue(data, len);
}


// BLE & GZLL

void  ble_rfduino_manageRadios() {

  static byte   state  = 0;
  static unsigned long lastTime = millis();
  unsigned long timeDelta = ( millis() - lastTime );

  /*
  if ( (millis() - lastGZLLPacketTime ) > 60000 ) {
      gzll_connected=false;
  }
  */

  if (ble_connected || gzll_connected) {
    return;
  }

  if (timeDelta > TOGGLE_MILLIS) {
    state++;
    lastTime =  millis();
  }

  if ( 0 == state) {
    state = 1;

    RFduinoGZLL.end();
    RFduinoBLE.customUUID = BLE_UUID;
    RFduinoBLE.deviceName = BLE_LOCALNAME;
    RFduinoBLE.advertisementData = BLE_ADVERTISEMENT_DATA;
    //RFduinoBLE.advertisementInterval(millis);
    
    RFduinoBLE.txPowerLevel= BLE_TX_POWER_LEVEL;
    RFduinoBLE.begin();
    DBG("RFd_BLE_ON",0);


  } else if (2 == state  ) {
    state = 3;
    while (!RFduinoBLE.radioActive);
    while (RFduinoBLE.radioActive);
    RFduinoBLE.end();
    while (RFduinoBLE.radioActive);
    RFduinoGZLL.txPowerLevel = GZLL_TX_POWER_LEVEL;
    RFduinoGZLL.begin(HOST);
    DBG("RFd_GZL_ON",0);
  } else if (4 == state) {
    state = 0;
  }

}



/////////////////////////////////////////////////////////
// Queue processing

void process_ble2uart_q() {
  if (radio2uartQItems>0) { 
    writeUART((uint8_t*) radio2uartQ[radio2uartQHead++], Q_ENTRY_SIZE);
    radio2uartQItems--;
    if (radio2uartQHead == Q_MAX_ENTRIES) {
      radio2uartQHead=0;
    }
  }  
}


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
  //pinMode(SS, OUTPUT);  
  //digitalWrite(SS, HIGH);  // ensure SS stays high for now


  SPI.begin ();

  // Slow down the master a bit
  SPI.setFrequency(125);
  // a no-op on RFduino SDK v2.0.3
  //SPI.setClockDivider(SPI_CLOCK_DIV8);

#endif // USE_SPI
  
  WATCHDOG_SETUP(2);
  DBG("",0);
  DBG("RFd_Setup",0);
}

void loop() {  
WATCHDOG_RELOAD() 
  ble_rfduino_manageRadios();
  process_ble2uart_q();
  process_uart2ble_q();
}
