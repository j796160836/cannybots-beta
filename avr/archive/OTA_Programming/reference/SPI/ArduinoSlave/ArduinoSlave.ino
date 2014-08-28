// Written by Nick Gammon
// February 2011


#include <SPI.h>
  
#define BUF_SIZE 100
char buf [BUF_SIZE];
volatile byte pos;
volatile boolean process_it;
volatile int intCount=0;

void setup (void)
{
  Serial.begin(9600);
  //delay(5000);
  Serial.println("CLIENT2!");
  //delay(1000);
  // have to send on master in, *slave out*
  pinMode(SCK, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS, INPUT);
  
  pinMode(13, OUTPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  
  // turn on interrupts
  //SPCR |= _BV(SPIE);
  
  // get ready for an interrupt 
  pos = 0;   // buffer empty
  process_it = false;

  // now turn on interrupts
  SPI.attachInterrupt();

}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;  // grab byte from SPI Data Register
  intCount++;
  
  // add to buffer if room
  if (pos < sizeof(buf))
  {
      buf [pos++] = c;      
    }  // end of room available
    
    /*
    if (c == '!' ) {
      process_it = true;
    }
    */
    if(pos==20) {
        process_it = true;
    }

}  // end of interrupt routine SPI_STC_vect

// main loop - wait for flag set in interrupt routine
void loop (void)
{
  //Serial.println(intCount,DEC);
  if (process_it)
    {
    digitalWrite(13,HIGH);
    //delay(200);
    digitalWrite(13,LOW);

    buf [pos] = 0;  
    Serial.println (buf);
    pos = 0;
    process_it = false;
    }  // end of flag set
    
    static int on = LOW;
    static unsigned long last = millis();
    if ((millis() - last) > 1000) {
       last =millis();
      digitalWrite(13, on = on==HIGH?LOW:HIGH); 
    }
}  // end of loop

