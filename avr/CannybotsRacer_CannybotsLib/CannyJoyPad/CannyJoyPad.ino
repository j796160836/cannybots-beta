
#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>

#include <Cannybots.h>


// SEE: http://www.rfduino.com/wp-content/uploads/2014/03/rfduino.ble_.programming.reference.pdf
// - RFduino_systemOff()
//
// pinMode(BUTTON_PIN, INPUT); // set pin 5 to input
// RFduino_pinWake(BUTTON_PIN, HIGH); // configures pin 5 to wake up device on a high signal


device_t role = DEVICE0;

#define XAXIS_PIN 2
#define YAXIS_PIN 3
#define BUTTON_PIN 4

#define AXIS_DEADZONE 25
#define AXIS_MIN_DELTA 2


// Axis state
int xAxisValue = 0, xAxisLastValue = 0;
int yAxisValue = 0, yAxisLastValue = 0;

// BUtton state

int buttonState;
int lastButtonState = LOW;
long lastDebounceTime = 0;
long debounceDelay = 50;

int followMode = 0;

unsigned long lastPing = millis();

void setup()
{
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  RFduinoGZLL.begin(role);
}

bool sendUpdate = false;

void loop()
{
  delay(100);
  sendUpdate = false;
  int buttonReading = digitalRead(BUTTON_PIN);
  xAxisValue = analogRead(XAXIS_PIN);
  yAxisValue = analogRead(YAXIS_PIN);


  xAxisValue = map(xAxisValue, 0, 1023, 255, -255);
  yAxisValue = map(yAxisValue, 0, 1023, 255, -255);

  if (buttonReading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonReading != buttonState) {
      buttonState = buttonReading;
      uint8_t msg[CB_MAX_MSG_SIZE] = {
        'C', 'B', 0,
        Cannybots::CB_INT16_2, 2 + 11, // RACER_LINEFOLLOWING_MODE
        1,
        0, buttonState == HIGH ? 1 : 0,
        0, 0,
        0, 0,
        0, 0,  0, 0,  0, 0, 0,0
      };

      RFduinoGZLL.sendToHost((const char*)msg, CB_MAX_MSG_SIZE);

    }
  }
  lastButtonState = buttonReading;

  // Detect X deadzone
  if ( abs(xAxisValue) < AXIS_DEADZONE)  {
    xAxisValue = 0; 
    /*if (xAxisLastValue != 0) {
      sendUpdate = true;
    } else {*/
       // after the immediate Axis centering send a throttled 0,0 every 200 ms
       static unsigned long lastSendXZeroTime = millis();
       if (millis()-lastSendXZeroTime > 500) {
        sendUpdate = true;
        lastSendXZeroTime = millis();
       }
 
    //}
  }
  if ( abs(yAxisValue) < AXIS_DEADZONE ) {
    yAxisValue = 0;
    /*if (yAxisLastValue != 0) {
      sendUpdate = true;
    } else {*/
       // after the immediate Axis centering send a throttled 0,0 every 200 ms
       static unsigned long lastSendYZeroTime = millis();
       if (millis()-lastSendYZeroTime > 500) {
        sendUpdate = true;
        lastSendYZeroTime = millis();
       }
 
    //}
  }
  
  
  if ( abs(xAxisLastValue - xAxisValue) > AXIS_MIN_DELTA) {
    sendUpdate = true;
  }
  if ( abs(yAxisLastValue - yAxisValue) > AXIS_MIN_DELTA) {
    sendUpdate = true;
  }
  xAxisLastValue = xAxisValue;
  yAxisLastValue = yAxisValue;


  if (sendUpdate) {
    sendUpdate = false;

    Serial.println("SendUpdate required");

    uint8_t msg[CB_MAX_MSG_SIZE] = {
      'C', 'B', 0,
      Cannybots::CB_INT16_2, 2 + 5, // SYSCALLS_MAX+RACER_JOYAXIS
      2,
      hiByteFromInt(xAxisValue), loByteFromInt(xAxisValue),
      hiByteFromInt(yAxisValue), loByteFromInt(yAxisValue),
      0, 0,
      0, 0,  0, 0,  0, 0, 0, 0
    };
#if 1
    Serial.print("x=");
    Serial.print(xAxisValue, DEC);
    Serial.print("\ty=");
    Serial.print(yAxisValue, DEC);
    Serial.print("\tbs=");
    Serial.print(buttonState, DEC);
    Serial.print("\tbr=");
    Serial.print(buttonReading, DEC);
    Serial.print("\tfollowMode=");
    Serial.print(followMode, DEC);
    Serial.println("");
#endif
    RFduinoGZLL.sendToHost((const char*)msg, CB_MAX_MSG_SIZE);
  }


  if (  (millis() - lastPing) > 200) {
    uint8_t ping[CB_MAX_MSG_SIZE] = {
      'C', 'B', 0,
      Cannybots::CB_INT16_2, 2 + 9, // SYSCALLS_MAX+RACER_PING
      0,
      0, 0,
      0, 0,
      0, 0,
      0, 0,  0, 0,  0, 0, 0, 0
    };
    //RFduinoGZLL.sendToHost("PING!");

    RFduinoGZLL.sendToHost((const char*)ping, CB_MAX_MSG_SIZE);
    lastPing=millis();
  }
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  // Serial.println("receive");
}
