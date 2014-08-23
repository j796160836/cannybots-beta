// pinout

// GPIO5 = SCL
// GPIO6 = SDA

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306ms.h>

#define OLED_RESET 0

//Adafruit_SSD1306 display(OLED_RESET);
Adafruit_SSD1306 display(OLED_RESET);


void setup()   {        
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  

  // Clear the buffer.
  display.clearDisplay();
  display.stopscroll();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20,0);
  display.println("CannyBots");
  display.setCursor(35,32);
  display.println("Racer!");
  //display.startscrolldiagright(0x00, 0x07);
  //display.startscrollleft(0,3);
  display.startscrollright(4,7);

  display.display();

}

void loop() {
  delay(100);
}
