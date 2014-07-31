/*
This sketch demonstrates how to send data from a Device
to a Host in a Gazell network.

When Button A on the Device is pressed and released,
the green led on the host will toggle.
*/

#include <RFduinoGZLL.h>
m
device_t role = DEVICE0;

// pin for Button A on the RGB Shield
int button_a = 5;

// debounce time (in ms)
int debounce_time = 10;

// maximum debounce timeout (in ms)
int debounce_timeout = 100;

// starting state is on
char state = 1;

void setup()
{
  Serial.begin(9600);
  pinMode(button_a, INPUT);

  // start the GZLL stack
  RFduinoGZLL.begin(role);
}

int debounce(int button, int state)
{
  int start = millis();
  int debounce_start = start;
  
  while (millis() - start < debounce_timeout)
    if (digitalRead(button) == state)
    {
      if (millis() - debounce_start >= debounce_time)
        return 1;
    }
    else
      debounce_start = millis();

  return 0;
}

int delay_until_button(int button, int state)
{
  while (! debounce(button, state))
    ;
}

void loop()
{
  delay_until_button(button_a, HIGH);
  
  // toggle state
   Serial.print("state: ");
   Serial.println(state, DEC);
  state = ! state;

  // send state to Host
  RFduinoGZLL.sendToHost(state);
  
  delay_until_button(button_a, LOW);
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  Serial.println("receive");
}
