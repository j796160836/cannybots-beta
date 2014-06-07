#include <stdint.h>
#include <RFduinoBLE.h>

template<class T> inline Print &operator <<(Print &obj, T arg) {  obj.print(arg);  return obj; }


void setup() {
  Serial.begin(9600);
  Serial << "start\n";  

  RFduinoBLE.customUUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
  // +0002, 6e400002-b5a3-f393-e0a9-e50e24dcca9e is TX (client listned)
  // +0003, 6e400003-b5a3-f393-e0a9-e50e24dcca9e is RX (client send)      both oposite to Nordic NUS lol
  // +0004, 6e400004-b5a3-f393-e0a9-e50e24dcca9e is 'something else' TBD...
  RFduinoBLE.begin();
}

void loop() {
  //RFduino_ULPDelay(INFINITE); // switch to lower power mode, does not return;
  
  char msg[] = {'h','e','l','l','o'};
  RFduinoBLE.send(msg, 5);
  delay(100);
  
  // Wait while the Radio is active 
  while (RFduinoBLE.radioActive)  ; 
  // Timing Critical Code goes here
}

void RFduinoBLE_onAdvertisement(bool start) {
  Serial << "advertisement: " << start << "\n";
}

void RFduinoBLE_onConnect() {
  Serial << "connect\n";  
}

void RFduinoBLE_onDisconnect() {
  Serial << "disconnect\n";  
}


void RFduinoBLE_onReceive(char *data, int len){ 
  uint8_t myByte = data[0]; // store first char in array to myByte 
  Serial.println(myByte, HEX); // print myByte via serial 
}
