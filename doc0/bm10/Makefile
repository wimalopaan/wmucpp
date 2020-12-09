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

F_OSC = 20000000

#MCU = atmega328pb
#MCU = atmega1284p
#MCU = atmega4809
MCU = attiny1614
#MCU = atxmega32e5


CPPFLAGS += -DNDEBUG
#CXXFLAGS += -fsanitize=address

#LDLIBS += -lasan

OPTFLAGS = -Os

targets += shtml

targets += bm00.S bm00.elf
targets += bm01.S bm01.elf
targets += bm02.S bm02.elf
targets += bm10.S bm10.elf

-include ../../Makefile.avr0.include

CXX = /usr/local/bin/avr-g++
