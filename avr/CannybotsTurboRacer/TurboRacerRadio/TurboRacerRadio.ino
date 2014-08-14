#include <RFduinoGZLL.h>
#define TX_PIN 5
#define RX_PIN 6
device_t role = HOST;

void setup() {
  Serial.begin(9600, RX_PIN, TX_PIN);        // UART Baud is limited to 9600 when the BLE stack is on.
  RFduinoGZLL.begin(role);
}

uint8_t buffer[5] = {0};
volatile bool send = false;

void loop() {
  if (send) {
    Serial.write(buffer, 5);
    //Serial.flush();
    send=false;
  }
  delay(50);
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  memcpy(buffer,data,min(5,len));
  send=true;
}
