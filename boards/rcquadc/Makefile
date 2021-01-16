
# ~/Projekte/updi/pyupdi/pyupdi.py -d atmega4808 -c /dev/ttyUSB1  -f test00.hex
#  .local/bin/pyupdi -d atmega4808 -c /dev/ttyUSB1  -f test00.hex

MCU = atmega4809
F_OSC = 20000000

#OPTFLAGS = -O3

subdirs =

test20.hex : MCU = avr128da32 
test20.hex : F_OSC = 32000000

#targets += test00.hex
#targets += test01.hex
#targets += test02.hex
#targets += test03.hex
#targets += test04.hex
#targets += test05.hex
#targets += test06.hex
#targets += test07.hex
#targets += test08.hex
#targets += test10.hex
#targets += test11.hex
#targets += test12.hex
#targets += test13.hex
#targets += test14.hex
#targets += test15.hex
#targets += test16.hex

targets += test20.hex

-include ../../Makefile.avr0.include

#CC = /usr/local/bin/avr-gcc
#CXX = /usr/local/bin/avr-g++
