#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>
#include <stdint.h>

// from: http://playground.arduino.cc/Code/QueueArray
#include <QueueArray.h>



#define TX_PIN 5
#define RX_PIN 6
#define BUF_MAX 20



// Inbound & outbound FIFO's
class BLEMessage {
  public:
    BLEMessage(uint8_t* buffer, uint16_t len) {
        size=len<BUF_MAX?len:BUF_MAX;
        memcpy(payload, buffer, size);
    };
  uint8_t payload[BUF_MAX];
  uint16_t size;
};

QueueArray <BLEMessage*> ble2uartMsgFIFO;

// double buffer
char radio2uart_buf0[2 + BUF_MAX] = { '>' , '>'};
char radio2uart_buf1[2 + BUF_MAX] = { '>' , '>'};

char* buffer0Ptr = radio2uart_buf0 +2;
char* buffer1Ptr = radio2uart_buf1 +2;
volatile char* bufferCurrentPtr  = buffer0Ptr;
volatile char* bufferReadPtr     = NULL;

void writeUART(const uint8_t* data, uint16_t len) {
  //while (!RFduinoBLE.radioActive);
  //while (RFduinoBLE.radioActive);
  Serial.write(">>");
  Serial.write( data, len);
  Serial.flush();              // crashes prior to rfduino 2.0.3
}

void RFduinoBLE_onReceive(char *data, int len) {
  if (bufferCurrentPtr == buffer0Ptr) {
    memcpy(buffer1Ptr, data, len <= BUF_MAX ? len : BUF_MAX);
    bufferCurrentPtr=buffer1Ptr;  
  } else if (bufferCurrentPtr == buffer1Ptr) {
    memcpy(buffer0Ptr, data, len <= BUF_MAX ? len : BUF_MAX);
    bufferCurrentPtr=buffer0Ptr;
  }
  bufferReadPtr=bufferCurrentPtr; 
}

void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);
  RFduinoBLE.customUUID = "7e400001-b5a3-f393-e0a9-e50e24dcca9e";
  RFduinoBLE.begin();
  
  // start the watchdog
  NRF_WDT->CRV = 32768 * 2; // Timeout period of 2 s
  NRF_WDT->TASKS_START = 1;
}

void loop() {
  // reload the watchdog timer
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
  
  if (bufferReadPtr) {
    //BLEMessage* msg = new BLEMessage((uint8_t*)radio2uart, sizeof(radio2uart));  
    writeUART((uint8_t*)bufferReadPtr, sizeof(radio2uart_buf0));
    bufferReadPtr=0;
    //bleReadDataReady=false;
    //bufferReadPtr=0;
    //ble2uartMsgFIFO.enqueue(msg);
  }

  /*
  // ble 2 UART
  if (!ble2uartMsgFIFO.isEmpty ()) {
    //Serial.print("size:");
    //Serial.println(ble2uartMsgFIFO.count(), DEC);
    BLEMessage* msg = ble2uartMsgFIFO.dequeue ();
    writeUART(msg->payload, msg->size);
    delete msg;
  }
  */
  
}
