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

MCU = atmega328p
F_OSC = 16000000
#F_OSC = 8000000

subdirs = 

targets +=  main.elf
targets +=  test.elf 
targets +=  testleds.elf

#test.elf: test.o simavrconsole.o
#	$(CC) $(LDFLAGS) -o $@ test.o simavrconsole.o
	

CXXFLAGS += -I../../../3rdparty

-include ../../../Makefile.include
