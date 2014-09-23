
#define  RADIO_ONLY_GZLL


#ifdef RADIO_ONLY_GZLL
void setup_radio() {
  //RFduinoGZLL.hostBaseAddress = 0x12ABCD12;
  RFduinoGZLL.begin(HOST);
}

// We're expecting messages of 3 bytes in the form:  XYB
// Where;
// X = unsigned byte for xAxis:          0 .. 255 mapped to -254 .. 254
// Y = unsigned byte for yAxis:          0 .. 255 mapped to -254 .. 254
// B = unsigned byte for button pressed: 0 = no, 1 = yes

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len) {
  if (len >= 3) {
    // map x&y values from 0..255 to -255..255
    joypad_update(
      map(data[0], 0, 255, -255, 255),   // x axis
      map(data[1], 0, 255, -255, 255),   // y axis
      data[2]                            // button
      );
  }
}
#endif
