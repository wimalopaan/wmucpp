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

# ~/Projekte/updi/pyupdi/pyupdi.py -d tiny412 -c /dev/ttyUSB1  -f test00.hex
# ~/.local/bin/pymcuprog write --erase -d attiny412 -t uart -u /dev/ttyUSB1 -f sbusOut.hex

MCU = attiny412
F_OSC = 20000000
# 3V Betrieb
#F_OSC = 10000000 

inc3mode10Mhz.hex: F_OSC = 10000000 
potiOnly00.hex: F_OSC = 5000000 

# subdirs = doc

#targets += test00.hex
#targets += test01.hex
targets += inc3mode10Mhz.hex
#targets += serialOnly.hex
targets += sbusOut.hex
targets += potiOnly00.hex

-include ../../Makefile.avr0.include

#CC = /usr/local/bin/avr-gcc
#CXX = /usr/local/bin/avr-g++
