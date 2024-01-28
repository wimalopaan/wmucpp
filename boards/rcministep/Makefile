
# ~/Projekte/updi/pyupdi/pyupdi.py -d tiny1614 -c /dev/ttyUSB1  -f test00.hex
# ~/.local/bin/pymcuprog write --erase -d attiny412 -t uart -u /dev/ttyUSB0 -f bldc_01.hex

MCU = attiny1614
F_OSC = 20000000

subdirs =

test412_00.hex: MCU = attiny412
test412_01.hex: MCU = attiny412

micro_01.hex: MCU = attiny412
bldc_01.hex: MCU = attiny412
#micro_01.hex: F_OSC = 10000000

#targets += test412_00.hex
#targets += test412_01.hex
#targets += test1614_00.hex
#targets += test1614_01.hex
targets += mini_01.hex
targets += micro_01.hex
targets += bldc_01.hex
#targets += test00.hex
#targets += test01.hex
#targets += test50.hex
#targets += test60.hex

-include ../../Makefile.avr0.include
