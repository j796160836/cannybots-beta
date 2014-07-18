// Written by Nick Gammon
// February 2011


#include <SPI.h>

void setup (void)
{
  Serial.begin(9600);
  Serial.println("SERVER!");

  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
#ifdef __RFduino__
  pinMode(SS, OUTPUT);  
  digitalWrite(SS, HIGH);  // ensure SS stays high for now
#endif

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
#ifdef __RFduino__
  //SPI.setFrequency(125);
#else
// a no-op on RFduino SDK v2.0.3
  SPI.setClockDivider(SPI_CLOCK_DIV8);
#endif  
  
}  // end of setup


void loop (void)
{

  char c;

  // enable Slave Select
  digitalWrite(SS, LOW);    // SS is pin 10

  // send test string
  Serial.println("Sending");
  for (const char * p = "Hello, world!\n" ; c = *p; p++)
    SPI.transfer (c);

  // disable Slave Select
  digitalWrite(SS, HIGH);

  delay (1000);  // 1 seconds delay 
}  // end of loop
