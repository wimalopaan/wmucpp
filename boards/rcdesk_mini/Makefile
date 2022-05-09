
# ~/Projekte/updi/pyupdi/pyupdi.py -d atmega4808 -c /dev/ttyUSB1  -f test00.hex
# ~/.local/bin/pymcuprog write --erase -d avr128da48 -t uart -u /dev/ttyUSB0 -f test10.hex

MCU = avr128da48
F_OSC = 32000000

subdirs =

targets += test00.hex test00.S
targets += test01.hex 
targets += test02.hex 
targets += test03.hex 
targets += test99.hex test99.S 
targets += desk10.hex desk10.elf

-include ../../Makefile.avr0.include
