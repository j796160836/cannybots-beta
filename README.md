cannybots-beta
==============

This repository contains the beta source code for Cannybots, including:

1  Cannybots robot line following

2. Cannybots Arduino and iOS libraries

3. Programming an Arduino Over the Air 

4. Historical and experimental code

by
Anish Mampetta
&
Wayne Keenan



Installation
------------

1. Versions

Arduino IDE:  Arduino 1.5.7



2. Arduino Library Dependancies

copy the folders under: cannybots-beta/avr/libraries
to: Documents/Arduino/libraries

You may need to create the folder, if so you might want to consider just copying the whole libraries folder from the git repo to the Arduiono sketchbook folder.


3. Arduin IDE:  Pololu A-Star support

You can either:

a) Set the board type to Leonardo 

OR

b) Install the Pololu add-on

Please follow the instruction here: http://www.pololu.com/docs/0J61/all#5.2 

The main benefit of the Pololu add-on is that the Pololu bootloader enables low-voltage (Vin <4.3v) detection by the user sketch after a reset.



4. Arduino IDE: Over-The-Air programming

1.  edit: [Arduino_INSTALL]/Java/hardware/arduino/avr/programmers.txt

e.g. on Windows:
e.g. on Mac: 		/Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/avr/programmers.txt

append:

rfduinoisp.name=RFduino as ISP (Cannybots)
rfduinoisp.communication=serial
rfduinoisp.protocol=avrisp
rfduinoisp.speed=9600
rfduinoisp.program.protocol=avrisp
rfduinoisp.program.speed=9600
rfduinoisp.program.tool=avrdude
rfduinoisp.program.extra_params=-P{serial.port} -b{program.speed}

2. Restart the IDE



iOS App
-------

http://cannybots.github.io/cannybots-beta/


