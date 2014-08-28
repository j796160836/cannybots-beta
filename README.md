#cannybots-beta

This repository contains the beta source code for CannyBots, including:

1. Cannybots robot line following
2. Cannybots Arduino and iOS libraries
3. Programming an Arduino Over the Air 
4. Historical and experimental code
5. RFduino Libray Patches

by
Anish Mampetta
&
Wayne Keenan


#Installation

##1. Versions

* Arduino IDE:  Arduino 1.5.7
* RFduino Lib:  v2.1


##2. Arduino Library Dependancies

If any folders exist under: 
* cannybots-beta/avr/libraries

copy them to: 

* Documents/Arduino/libraries

You may need to create the folder, if so you might want to consider just copying the whole libraries folder from the git repo to the Arduiono sketchbook folder.


##3. Arduino IDE:  Pololu A-Star support

You can either:

* Set the board type to Leonardo 

OR

* Install the Pololu add-on

Please follow the instruction here: http://www.pololu.com/docs/0J61/all#5.2 

The main benefits of the Pololu add-on is that the Pololu bootloader enables:
* low-voltage (Vin <4.3v) detection by the user sketch after a reset.
* forcing the AVR into bootloader mode then connecting RST to GND twice within 750 ms after power on




##4. Arduino IDE RFduino Support

a) Install RFduino v2.1 Library for Arduino

Download and install the v2.1 of the RFduino library:  

* http://www.rfdigital.com/wp-content/uploads/2014/03/RFduino_2.1.zip

It *MUST* be  v2.1 because of some customisations we've made. 
You also must be using ARduino IDE 1.5.7 or higher for the RFduino library to work.

Unzip the folder under:  [ArduinoIDE_Install_PATH]/Java/hardware/arduino/


b) custom Arduino Gazelle libraries

you will find 2 zips in this git repo under :  

* cannybots-beta/avr/patches/RFduino/

these need to replace sub-folders under the RFduino folder that was installed into the Arduino IDE in the previous step.

* RFduinoGZLL.zip			unzip under:  	[ArduinoIDE_Install_PATH]/Java/hardware/arduino/RFduino/libraries
* libRFduinoGZLL.zip		unzip under:	[ArduinoIDE_Install_PATH]/Java/hardware/arduino/RFduino/source

It's preferable to move or just delete the existing folders rather than renaming them in-place:

* [ArduinoIDE_Install_PATH]/Java/hardware/arduino/RFduino/libraries/RFduinoGZLL
* [ArduinoIDE_Install_PATH]/Java/hardware/arduino/RFduino/libraries/libRFduinoGZLL

For convenience these are the direct links to the zips in the online repo: 

* https://github.com/Cannybots/cannybots-beta/blob/master/avr/patches/RFduino/RFduinoGZLL.zip
* https://github.com/Cannybots/cannybots-beta/blob/master/avr/patches/RFduino/libRFduinoGZLL.zip


##5. Arduino IDE: Over-The-Air programming

a).  edit: [Arduino_INSTALL]/Java/hardware/arduino/avr/programmers.txt

e.g. on Windows:
e.g. on Mac: 		/Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/avr/programmers.txt

append:

```
rfduinoisp.name=RFduino as ISP (Cannybots)
rfduinoisp.communication=serial
rfduinoisp.protocol=avrisp
rfduinoisp.speed=9600
rfduinoisp.program.protocol=avrisp
rfduinoisp.program.speed=9600
rfduinoisp.program.tool=avrdude
rfduinoisp.program.extra_params=-P{serial.port} -b{program.speed}
```

b). Restart the IDE



#iOS App

http://cannybots.github.io/cannybots-beta/


