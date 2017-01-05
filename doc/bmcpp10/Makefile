# -*- mode: makefile-gmake; -*-
#
# WMuCpp - Bare Metal C++ 
# Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#MCU = atmega1284p
F_OSC = 20000000

bm01a.elf : LOADLIBES = simavrconsole.o

targets += simavrconsole.o
targets +=  bm01a.s bm01a.elf 

-include ../../Makefile.include

#bm01a.elf: bm01a.o simavrconsole.o
#	$(CC) $(LDFLAGS) -o $@ bm01a.o simavrconsole.o


