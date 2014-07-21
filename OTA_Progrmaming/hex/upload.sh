#DEV=/dev/cu.usbmodem26221
DEV=/dev/tty.usbserial-DC008J1H

#BAUD=57600
#BAUD=19200
BAUD=9600

#APP=Arduino-0022.app
#APP=Arduino-1.0.5.app
#APP=Arduino-1.5.7.app
APP=Arduino-1.5.6r2.app

BIN=/Applications/$APP/Contents/Resources/Java/hardware/tools/avr/bin/avrdude
CONF=/Applications/$APP/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf

file $CONF


PROTO=avrisp
#PROTO=arduino
#PROTO=stk500v2

VERBOSE='-v -v -v -v'
VERBOSE='-v -v -v -v'

#bootlaoder


$BIN -C$CONF $VERBOSE -patmega32u4 -cstk500v1 -P$DEV -b$BAUD -e -Ulock:w:0x3F:m -Uefuse:w:0xc8:m -Uhfuse:w:0xd0:m -Ulfuse:w:0xff:m

#flash
$BIN -C$CONF $VERBOSE -patmega32u4 -c $PROTO -P$DEV -b$BAUD -D -Uflash:w:./Blink.cpp.hex:i


