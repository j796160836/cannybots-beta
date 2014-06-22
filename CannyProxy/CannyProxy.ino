#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>
#include <SimpleFIFO.h>
#include <NTProtocol.h>
#include <NTUtils.h>
#include <NTMessaging.h>

#define DBG(x)
//#define LED_DEBUG
// A-star pin 0 RX
// A-star pin 1 TX

#define TX_PIN 5
#define RX_PIN 6

// connect RFd TX pin to WHITE PL2303 USB TTL Serial lead
// connect RFd RX pin to GREEN PL2303 USB TTL Serial lead



// loop ticks to swap between BLE and GZLL
#define TOGGLE_MILLIS 2500




// Inbound & outbound FIFO's

NTQueue ble2uartMsgFIFO;
NTQueue uart2bleMsgFIFO;

boolean volatile ble_connected = false;
boolean volatile gzll_connected = false;

void RFduinoBLE_onAdvertisement(bool start) {
}

void RFduinoBLE_onConnect() {
  DBG("RFduinoBLE_onConnect");
  ble_connected = true;
#ifdef LED_DEBUG
  digitalWrite(2, HIGH);
#endif


}

void RFduinoBLE_onDisconnect() {
#ifdef LED_DEBUG
  digitalWrite(2, LOW);
#endif
  DBG("RFduinoBLE_onDisconnect");
  ble_connected = false;
}

volatile bool sending = false;

void RFduinoBLE_onReceive(char *data, int len) {
  DBG("RFduinoBLE_onReceive");
  if (sending)
    return;
  BLEMessage* msg = new BLEMessage((uint8_t*)data, len);
  ble2uartMsgFIFO.enqueue(msg);
}


// GZLL
void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
#ifdef LED_DEBUG
  static bool set = false;
  if (!set) {
    digitalWrite(3, HIGH);
    set = true;
  }
#endif

  DBG("RFduinoGZLL_onReceive");

  gzll_connected = true;
  BLEMessage* msg = new BLEMessage((uint8_t*)data, len);
  ble2uartMsgFIFO.enqueue(msg);

  // MAYBE-TODO: check uart2ble here as Gazelle can only piggy back client device requests
  //RFduinoGZLL.sendToDevice(device, "OK");
}

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
    DBG("BLE");

    RFduinoGZLL.end();
    RFduinoBLE.customUUID = "7e400001-b5a3-f393-e0a9-e50e24dcca9e";
    RFduinoBLE.deviceName = "Cannybots";
    RFduinoBLE.advertisementData = "Leebo";
    RFduinoBLE.begin();

  } else if (2 == state  ) {
    state = 3;
    DBG("GZLL");
    while (!RFduinoBLE.radioActive);
    while (RFduinoBLE.radioActive);
    RFduinoBLE.end();
    while (RFduinoBLE.radioActive);
    RFduinoGZLL.begin(HOST);
  } else if (4 == state) {
    state = 0;
  }

}



// scedule BLE to UART message
// see callback above

// scedule UART to BLE message

uint8_t  sendBLE(uint8_t* buffer) {
  BLEMessage* msg = new BLEMessage(buffer, NT_MSG_SIZE);
  uart2bleMsgFIFO.enqueue(msg);
}


// Message queue processing

void processBLE2UARTMessageQueue() {
  sending = true;
  for (int i = 0; i < ble2uartMsgFIFO.count(); i++) {
    BLEMessage* msg = ble2uartMsgFIFO.dequeue();
    Serial.write(">>");
    Serial.write(msg->payload, msg->size);
    delete msg;
  }
  sending = false;
}

void processUART2BLEMessageQueue() {
  for (int i = 0; i < uart2bleMsgFIFO.count(); i++) {
    BLEMessage* msg = uart2bleMsgFIFO.dequeue();

    if (ble_connected) {
      while (!RFduinoBLE.radioActive);
      while (RFduinoBLE.radioActive);
      RFduinoBLE.send((char*)msg->payload, NT_MSG_SIZE);
    } else if (gzll_connected) {
      // can only piggy back, so need to send these as part of a client request , see RFduinoGZLL_onReceive()
    } else {
      // discard message as we are not connected to anything.
    }

    delete msg;
  }
}





/////////////

void setup() {
#ifdef LED_DEBUG
  for (int i = 1 ; i <= 3; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH); delay(500);
    digitalWrite(i, LOW);
  }
  digitalWrite(1, HIGH);
#endif
  Serial.begin(9600, RX_PIN, TX_PIN);
  DBG("CannyProxy UP!");
  delay(2000);
}

#define NT_CAT_APP_LINEFOLLOW NT_CAT_APP
#define NT_CMD_LINEFOLLOW_MOVE 1
#define LINEFOLLOW_RIGHT 4


void sendHeartBeat () {
  static unsigned long lastTime = millis();
  if (millis() - lastTime > 1000) {
    lastTime = millis();

    uint8_t data[NT_MSG_SIZE] = {
      NT_DEFAULT_MSG_HEADER(),
      NT_CREATE_CMD1(NT_CAT_APP_LINEFOLLOW, NT_CMD_LINEFOLLOW_MOVE, LINEFOLLOW_RIGHT, 0),
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP,
      NT_CREATE_CMD_NOP
    };
    BLEMessage* msg = new BLEMessage((uint8_t*)data, NT_MSG_SIZE);
    ble2uartMsgFIFO.enqueue(msg);
  }
}

void loop() {
  NT_UART_parseIntoQueue(Serial, uart2bleMsgFIFO);
  sendHeartBeat();
  processBLE2UARTMessageQueue();
  ble_rfduino_manageRadios();
  processUART2BLEMessageQueue();
}
