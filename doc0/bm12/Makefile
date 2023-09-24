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

#MCU = atmega328p
MCU = avr128da28

CPPFLAGS += -DNDEBUG

OPTFLAGS = -Os
targets += shtml

targets += bm00.S bm00.elf bm00.hex
targets += bm01.S bm01.elf bm01.hex
targets += bm02.S bm02.elf bm02.hex
targets += bm03.S bm03.elf bm03.hex
targets += bm04.S bm04.elf bm04.hex
targets += bm05.S bm05.elf bm05.hex
targets += bm06.S bm06.elf bm06.hex
targets += bm07.S bm07.elf bm07.hex
targets += bm08.S bm08.elf bm08.hex
targets += bm09.S bm09.elf bm09.hex

-include ../../Makefile.avr0.include

#CXX = /usr/local/bin/avr-g++
#CC = /usr/local/bin/avr-gcc
CXX = /usr/bin/avr-g++
CC = /usr/bin/avr-gcc
