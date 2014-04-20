
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);


/**************************************************************************/
/*!
    This function is called whenever select ACI events happen
*/
/**************************************************************************/
void aciCallback(aci_evt_opcode_t event)
{
  switch(event)
  {
    case ACI_EVT_DEVICE_STARTED:
      //Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      //Serial.println(F("Connected!"));
      break;
    case ACI_EVT_DISCONNECTED:
      //Serial.println(F("Disconnected or advertising timed out"));
      break;
    default:
      break;
  }
}




#include <ax12.h>

AX12 motor1, motor2;

long readVcc() {
  // Read 1.1V reference against AVcc. Inverting that, means vcc becomes known!
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  // call this only in setup()
  // ref    http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

  uint8_t admux_0 = ADMUX;   // save it.   presumably =0?
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);   // aref=vcc read=1.1 internal
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
  ADMUX = admux_0;   // restore
 
  long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
int voltage = 0;

byte detect[2] = {0};
byte num=0;            

void setup() {
  voltage = readVcc();
  AX12::init (1000000);   // inicializa los AX12 a 1 Mb/s
  num = AX12::autoDetect (detect, 2); 
  int m1id = num>0? detect[0]: 1; 
  int m2id = num>1? detect[1]: 50; 
  motor1 = AX12((byte)m1id);
  motor2 = AX12((byte)m2id,true);    // true for inverted commands, motors are facing each other
  
  motor1.setEndlessTurnMode(true);
  motor2.setEndlessTurnMode(true);
  
  uart.setRXcallback(rxCallback);
  uart.setACIcallback(aciCallback);
  uart.begin();
}



void rxCallback(uint8_t *buffer, uint8_t len)
{
  int speed = 0;
      
  char str[128];
  switch (buffer[0]) {
    case 'i':
          uart.println("changeid");
          motor1.changeID (buffer[1]-48); // TOOD: change this to the 'broadcast' ID (254)
          break;
    case 'v':
          uart.println("setVel");
          //motor.setVel (200);
          break;
    case 'p':
          uart.println("setPos");
          //motor.setPos ((buffer[0]-47) * 100);
          break;
    case 'b':
          uart.println("backward");
          speed = (buffer[1]-48)*100;
          motor1.endlessTurn(-speed);
          motor2.endlessTurn(-speed);          break;
    case 'f':
          uart.println("forward");
          speed = (buffer[1]-48)*100;
          motor1.endlessTurn(speed);
          motor2.endlessTurn(speed);
          break;
          
    default:
        uart.println("Brain is alive!");
        sprintf(str, "Voltage (read only on startup): %d mv", voltage);
        uart.println(str);
        sprintf(str, "Detected: %d", num);
        uart.println(str);
        sprintf(str, "ping 1: %s" , motor1.ping()?"Err":"OK");
        uart.println(str);
        sprintf(str, "ping 2: %s" , motor2.ping()?"Err":"OK");
        uart.println(str);
        break;
  }
  uart.write(buffer, len);
}

void loop() {
    uart.pollACI();
}
