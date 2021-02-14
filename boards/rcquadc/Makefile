
# ~/Projekte/updi/pyupdi/pyupdi.py -d atmega4808 -c /dev/ttyUSB1  -f test00.hex
#  .local/bin/pyupdi -d atmega4808 -c /dev/ttyUSB1  -f test00.hex

MCU = atmega4809
F_OSC = 20000000

#OPTFLAGS = -O3

subdirs =

test20.hex : MCU = avr128da32 
test20.hex : F_OSC = 32000000

avr128da32_01.hex : MCU = avr128da32 
avr128da32_01.hex : F_OSC = 32000000

avr128da32_02.hex : MCU = avr128da32 
avr128da32_02.hex : F_OSC = 32000000

avr128da32_04.hex : MCU = avr128da32 
avr128da32_04.hex : F_OSC = 32000000

test30.hex : MCU = avr128da32 
test30.hex : F_OSC = 32000000

test31.hex : MCU = avr128da32 
test31.hex : F_OSC = 32000000

eeprom.hex : MCU = avr128da32 
eeprom.hex : F_OSC = 32000000

eeprom2.hex : MCU = avr128da32 
eeprom2.hex : F_OSC = 24000000

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
#targets += eeprom.hex
#targets += eeprom2.hex
#targets += test30.hex
#targets += test31.hex

# HW 02 (erste Version)
#targets += test16.hex
targets += atmega4808_01.hex

#targets += test20.hex
targets += avr128da32_01.hex

# HW 03 (hat noch Fehler)
targets += avr128da32_02.hex

# HW 04

targets += avr128da32_04.hex

#targets += test99.hex

-include ../../Makefile.avr0.include

#CC = /usr/local/bin/avr-gcc
#CXX = /usr/local/bin/avr-g++
