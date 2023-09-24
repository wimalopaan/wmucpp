# -*- mode: makefile-gmake; -*-
#
# WMuCpp - Bare Metal C++ 
# Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# ~/Projekte/updi/pyupdi/pyupdi.py -d tiny1614 -c /dev/ttyUSB1  -f bm01.hex

F_OSC = 24000000

#MCU = atmega328pb
#MCU = atmega1284p
#MCU = atmega4809
#MCU = atmega328p
MCU = avr128da48
#MCU = atxmega32e5

bm01.s: MCU = atmega328p

bm00.s: MCU = atmega328p
bm00.S: MCU = atmega328p
bm00.elf: MCU = atmega328p
bm00.hex: MCU = atmega328p

bm42.s: MCU = atmega8
bm42.elf: MCU = atmega8

bm60.s: MCU = atmega328p
bm60.elf: MCU = atmega328p

bm65.s: MCU = atmega328p
bm65.elf: MCU = atmega328p

bm66.s: MCU = atmega328p
bm66.elf: MCU = atmega328p

bm61.s: MCU = atmega328p
bm61.elf: MCU = atmega328p

CXXFLAGS += -pedantic -Wextra # -fstrict-enums
CFLAGS += -pedantic

CPPFLAGS += -DNDEBUG
#CXXINLUDES4 = -I /home/lmeier/Projekte/cnl/cnl/include

OPTFLAGS = -Os

targets += shtml

targets += bm00.S bm00.elf bm00.hex
targets += bm01.S 
#targets += bm02.S bm02.elf bm02.hex
#targets += bm10.S bm10.elf bm10.hex
#targets += bm20.S bm20.elf bm20.hex
#targets += bm30.S bm30.elf bm30.hex
targets += bm40.S
targets += bm41.S
targets += bm42.S bm42.elf
targets += bm50.S bm50.elf
targets += bm51.S bm51.elf
targets += bm52.S bm52.elf
#targets += bm52
targets += bm60.S bm60.elf
targets += bm61.S bm61.elf
targets += bm62.S bm62.elf
targets += bm63.S bm63.elf
targets += bm65.S bm65.elf
targets += bm66.S bm66.elf
targets += bm70.S bm70.elf
targets += bm71.S bm71.elf
targets += bm80.S bm80.elf
targets += bm90.S bm90.elf
targets += bm91.S bm91.elf
targets += bm92.S bm92.elf

bm52: bm52.o bm52b.o 

bm52.o: bm52.c bm52.h

bm52b.o: bm52b.c bm52.h

#bm00.o: bm00.o bm00b.o

#bm40.o: bm40.o bm41.o 

#bm40a: bm40.o bm41.o

#targets += bm02.S bm02.elf

-include ../../Makefile.avr0.include

CXX = /usr/local/bin/avr-g++
CC = /usr/local/bin/avr-gcc
