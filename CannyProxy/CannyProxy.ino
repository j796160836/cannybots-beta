#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>
#include <stdint.h>

/////////////////////////////////////////////////////////
// Config

#define BLE_UUID                 "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_LOCALNAME            "Cannybots"
#define BLE_ADVERTISEMENT_DATA   "CB_LFB_001"

#define TOGGLE_MILLIS 2500

#define Q_MAX_ENTRIES 12
#define Q_ENTRY_SIZE 20

#define UART_SOURCE Serial
#define SERIAL_BUF_SIZE 32

#define NT_MSG_SIZE 20

#define TX_PIN 5
#define RX_PIN 6


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
  Serial.write(">>");
  Serial.write( data, len);
  Serial.flush();              // crashes prior to rfduino 2.0.3
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
void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  gzll_connected = true;
  enqueue(data, len);
  // TODO: fake pairing. check the device is the same as before...

  // MAYBE-TODO: check uart2ble here as Gazelle can only piggy back client device requests
  //RFduinoGZLL.sendToDevice(device, "OK");
}


// BLE
void RFduinoBLE_onAdvertisement(bool start) {
}

void RFduinoBLE_onConnect() {
  ble_connected = true;
}

void RFduinoBLE_onDisconnect() {
  ble_connected = false;
}

void RFduinoBLE_onReceive(char *data, int len) {
  enqueue(data, len);
}


// BLE & GZLL

void  ble_rfduino_manageRadios() {

  static byte   state  = 0;
  static unsigned long lastTime = millis();
  unsigned long timeDelta = ( millis() - lastTime );

  if (ble_connected)
    return;
  if (gzll_connected)
    return;

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
    RFduinoBLE.begin();

  } else if (2 == state  ) {
    state = 3;
    while (!RFduinoBLE.radioActive);
    while (RFduinoBLE.radioActive);
    RFduinoBLE.end();
    while (RFduinoBLE.radioActive);
    RFduinoGZLL.begin(HOST);
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
  Serial.begin(9600, RX_PIN, TX_PIN);        // can, but shouldn't, go higher than 9600 (RFduino limitation)
  NRF_WDT->CRV = 32768 * 2;   // Timeout period of 2 s
  NRF_WDT->TASKS_START = 1;   // start the watchdog 
}

void loop() {  
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;    // reload the watchdog timer
  ble_rfduino_manageRadios();
  process_ble2uart_q();
  process_uart2ble_q();
}
