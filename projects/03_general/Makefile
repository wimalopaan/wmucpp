# -*- mode: makefile-gmake; -*-
#
# WMuCpp - Bare Metal C++ 
# Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
F_OSC = 20000000

targets +=  spiusart.elf spiusart.size
targets +=  simavr.elf simavr.size

spiusart.elf spiusart.pgr: MCU = atmega328p
spiusart.elf spiusart.pgr: F_OSC = 16000000

simavr.elf: LDFLAGS += "-Wl,--section-start=.mmcu=0x910000" # special flags for simavr
#simavr.elf: MCU = atmega1284p
simavr.elf: MCU = atmega328p
simavr.elf: F_OSC = 20000000
simavr.elf: simavr.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ simavr.o simavrconsole.o


-include ../../Makefile.include

