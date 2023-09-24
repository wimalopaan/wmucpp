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

targets += EncoderDemo.S EncoderDemo.elf EncoderDemo.hex

-include ../../Makefile.avr0.include

#CXX = /usr/local/bin/avr-g++
#CC = /usr/local/bin/avr-gcc
CXX = /usr/bin/avr-g++
CC = /usr/bin/avr-gcc
