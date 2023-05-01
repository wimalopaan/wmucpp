
# ~/Projekte/updi/pyupdi/pyupdi.py -d atmega4808 -c /dev/ttyUSB1  -f test00.hex
# ~/.local/bin/pymcuprog write --erase -d avr128da48 -t uart -u /dev/ttyUSB0 -f test10.hex

MCU = avr128db64
F_OSC = 32000000

subdirs =

# AVR128ProtoBoard
# Devices: HWRev = 0
#targets += test00.hex test00.S
#targets += test01.hex test01.S
#targets += test02.hex test02.S
#targets += test03.hex test03.S
#targets += test04.hex test04.S
#targets += test05.hex test05.S

# AVR128ProtoBoard_2
# Devices: HWRev = 1
#targets += test2_00.hex test2_00.S
#targets += test2_01.hex test2_01.S
#targets += test2_02.hex test2_02.S
#targets += test2_03.hex test2_03.S
#targets += test2_04.hex test2_04.S
#targets += test2_05.hex test2_05.S
#targets += test2_06.hex test2_06.S
#targets += test2_07.hex test2_07.S
#targets += test2_08.hex test2_08.S

# AVR128ProtoBoard_3
# Devices: HWRev = 2
#targets += test3_09.hex test3_09.S
#targets += test3_10.hex test3_10.S
#targets += test3_11.hex test3_11.S
#targets += test3_12.hex test3_12.S

# AVR128ProtoBoard_5
# Devices: HWRev = 4
targets += test5_12.hex test5_12.S
targets += test5_20.hex test5_20.S
targets += client

client:
	make -f Makefile.client all
	
-include ../../Makefile.avr0.include

CC = /usr/local/bin/avr-gcc
CXX = /usr/local/bin/avr-g++
