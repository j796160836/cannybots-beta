
//#define  RADIO_ONLY_GZLL
#define  RADIO_ONLY_BLE


#define BLE_UUID                   "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_ADVERTISEMENT_DATA_MAX 16
#define BLE_TX_POWER_LEVEL         0


void radio_setup() {
  
#if defined(RADIO_ONLY_GZLL)
  setup_gzll();
#elif defined(RADIO_ONLY_BLE)
  setup_ble();
#else
// BLE/GZLL toggling mode
#endif
}

void radio_loop() {
}

////////////////////////////////////////////////////////////////////////////////////////
// GZLL 

void setup_gzll() {
  //RFduinoGZLL.hostBaseAddress = 0x12ABCD12;
  RFduinoGZLL.begin(HOST);
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len) {
  process_message(data, len);
}

////////////////////////////////////////////////////////////////////////////////////////
// BLE 

char bleName[BLE_ADVERTISEMENT_DATA_MAX] = {0};

void setup_ble() {
  snprintf(bleName, BLE_ADVERTISEMENT_DATA_MAX, "CB_%x%x", getDeviceIdHigh, getDeviceIdLow());
  RFduinoBLE.txPowerLevel      = BLE_TX_POWER_LEVEL;
  RFduinoBLE.customUUID        = BLE_UUID;
  RFduinoBLE.deviceName        = bleName;
  RFduinoBLE.advertisementData = bleName;
  RFduinoBLE.begin();
}


void RFduinoBLE_onReceive(char *data, int len) {
  process_message(data, len);
}

void RFduinoBLE_onConnect() {
#ifdef DEBUG
  Serial.println("BLE_CON");
#endif
}

void RFduinoBLE_onDisconnect() {
#ifdef DEBUG
  Serial.println("BLE_DCON");
#endif
}


////////////////////////////////////////////////////////////////////////////////////////
// BLE/GZLL shared message processing

// We're expecting messages of 3 bytes in the form:  XYB
// Where;
// X = unsigned byte for xAxis:          0 .. 255 mapped to -254 .. 254
// Y = unsigned byte for yAxis:          0 .. 255 mapped to -254 .. 254
// B = unsigned byte for button pressed: 0 = no, 1 = yes

void process_message(char *data, int len) {  
  if (len >= 3) {
    // map x&y values from 0..255 to -255..255
    joypad_update(
      map(data[0], 0, 255, -255, 255),   // x axis
      map(data[1], 0, 255, -255, 255),   // y axis
      data[2]                            // button
      );
  }
}

