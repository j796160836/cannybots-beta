DEV=/dev/tty.usbmodem621
APP=Arduino-1.5.6r2.app

BIN=/Applications/$APP/Contents/Resources/Java/hardware/tools/avr/bin/avrdude
CONF=/Applications/$APP/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf

PROTO=avr109
VERBOSE='-v -v -v -v'


$BIN -C$CONF $VERBOSE -patmega32u4 -c $PROTO -P$DEV  -D -Uflash:w:./Blink.cpp.hex:i -e

