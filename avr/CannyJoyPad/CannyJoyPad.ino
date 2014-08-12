
#include <RFduinoBLE.h>
#include <RFduinoGZLL.h>

#include <Cannybots.h>

// TODO: include shared common line following header, once created.
#define NT_CAT_APP_LINEFOLLOW NT_CAT_APP
#define NT_CMD_LINEFOLLOW_MOTOR_SPEED 4

// for follow/dont follow
#define NT_CMD_LINEFOLLOW_MOVE 1
#define LINEFOLLOW_STOP 1
#define LINEFOLLOW_GO 2

// SEE: http://www.rfduino.com/wp-content/uploads/2014/03/rfduino.ble_.programming.reference.pdf
// - RFduino_systemOff()
//
// pinMode(BUTTON_PIN, INPUT); // set pin 5 to input
// RFduino_pinWake(BUTTON_PIN, HIGH); // configures pin 5 to wake up device on a high signal


device_t role = DEVICE0;

#define XAXIS_PIN 2
#define YAXIS_PIN 3
#define BUTTON_PIN 4

#define AXIS_DEADZONE 5
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

void setup()
{
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT);
  RFduinoGZLL.begin(role);
}

bool sendUpdate = false;

void loop()
{
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
      if (buttonState == HIGH) {
        
        //RFduinoGZLL.sendToHost((const char*)msg, NT_MSG_SIZE);
      } else {
        
      }
    }
  }
  lastButtonState = buttonReading;

  if ( ( abs(xAxisValue) < AXIS_DEADZONE) && ( abs(yAxisValue) < AXIS_DEADZONE) ) {
    xAxisValue = yAxisValue = 0;
    if (xAxisLastValue != 0 && yAxisLastValue != 0) {
      sendUpdate = true;
    }
  } else {


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

    //Serial.println("SendUpdate required");

    uint8_t msg[CB_MAX_MSG_SIZE] = {
        'C', 'B', 0,
        Cannybots::CB_INT16_2, 5,  // RACER_JOYAXIS
        2,
        hiByteFromInt(xAxisValue),loByteFromInt(xAxisValue),
        hiByteFromInt(yAxisValue),loByteFromInt(yAxisValue),
        0,0,
        0,0,  0,0,  0,0, 0,0
    };


    RFduinoGZLL.sendToHost((const char*)msg, sizeof(msg));
  }
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
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  // Serial.println("receive");
}
