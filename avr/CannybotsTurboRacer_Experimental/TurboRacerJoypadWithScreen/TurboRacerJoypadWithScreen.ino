//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Cannybots Joypad
//
// Author:  Wayne Keenan
//
// License: http://opensource.org/licenses/MIT
//
// Version:   1.0   -  14.08.2014  -  Inital Version
//
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <RFduinoGZLL.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306ms.h>

#define OLED_RESET 0

// Pins:
#define XAXIS_PIN 2
#define YAXIS_PIN 3
#define BUTTON_PIN 4
// PIN 5 = SCL
// PIN 6 = SDA


#define MSG_LEN 15        // 14 GZLL pack + 1 NULL byte during packet construction usin snprintf


device_t role = DEVICE0;
char msg[MSG_LEN] = {0};
Adafruit_SSD1306 display(OLED_RESET);

int lapTime = 0;
int laps    = 0;
int lastLap = 0;


void setup()
{
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT);
  //RFduinoGZLL.hostBaseAddress=0x0D0A0704;  // NRF default
  //RFduinoGZLL.deviceBaseAddress0x0E0B0805; // NRF default
  RFduinoGZLL.hostBaseAddress = 0x91827332;
  RFduinoGZLL.begin(role);

  display_setup();
}

void loop()
{
  bool buttonPressed = digitalRead(BUTTON_PIN) == HIGH;
  int xAxis = map( analogRead(XAXIS_PIN), 0, 1023, 255, -255);
  int yAxis = map( analogRead(YAXIS_PIN), 0, 1023, 255, -255);

  snprintf(msg, MSG_LEN, "%c%c%s%c%c%c%c%c%c%c", 0, 1, "JOY01", 6, highByte(xAxis), lowByte(xAxis), highByte(yAxis), lowByte(yAxis), highByte(buttonPressed), lowByte(buttonPressed));

  RFduinoGZLL.sendToHost((const char*)msg, MSG_LEN - 1);    // don't bother sending the NULL byte at the end
  //Serial.write((uint8_t*)msg, MSG_LEN - 1);
  //Serial.print("  ");
  delay(50);

  if (lastLap != laps) {
    display_dash(lapTime, laps);
    printStats(lapTime, laps);
    lastLap = laps;
  }
}

void display_setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.stopscroll();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20, 0);
  display.println("CannyBots");
  display.setCursor(0, 32);
  display.println("Racer!");
  //display.startscrolldiagright(0x00, 0x07);
  //display.startscrollleft(0,3);
  display.startscrollright(4, 7);

  display.display();
  delay(2000);
  display.clearDisplay();
  display.stopscroll();
  display.display();
  display_dash(0, 0);
}

void display_dash(int time, int laps) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Laps:");
  display.setCursor(0, 32);
  display.println("Time:");

  display.setTextSize(2);
  display.setCursor(48, 0);
  display.println(laps);
  display.setCursor(48, 32);
  display.println(time / 1000.0, 2);

  display.display();

}

void printStats(int _time, int _laps) {
  Serial.print("Lap ");
  Serial.print(_laps);
  Serial.print(", time=");
  Serial.println(_time / 1000.0, 2);
}



void RFduinoGZLL_onReceive(device_t device, int rssi, char * data, int len)
{
  if (len >= 12) {
    lapTime = (data[8] << 8)  + data[9];
    laps = (data[10] << 8) + data[11];
  }
}
