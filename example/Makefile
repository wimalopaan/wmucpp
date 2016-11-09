# -*- mode: makefile-gmake; -*-
# $Id: Makefile.include 1209 2016-07-01 07:53:57Z wimalopaan $
#
# ++C - C++ introduction
# Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

MCU = atmega1284p
F_OSC = 20000000

targets +=  main.elf main.size spiusart.elf spiusart.size
#targets +=  main.elf main.hex main.size spiusart.elf spiusart.hex spiusart.size
targets +=  simavr.elf simavr.size
#targets +=  isrtest1.elf isrtest1.size test1.elf test1.size test2.elf test2.size test5.elf test5.size
#targets +=  test10.elf test10.size test11.elf test11.size test12.elf test12.size
#targets +=  spiusart.elf spiusart.hex spiusart.size

spiusart.elf spiusart.pgr: MCU = atmega328p
spiusart.elf spiusart.pgr: F_OSC = 16000000

simavr.elf: LDFLAGS += "-Wl,--section-start=.mmcu=0x910000" # special flags for simavr
simavr.elf: MCU = atmega328p
simavr.elf: F_OSC = 20000000
simavr.elf: simavr.o simavrconsole.o flash.o
	$(CC) $(LDFLAGS) -o $@ simavr.o simavrconsole.o flash.o

test10.elf: LDFLAGS += "-Wl,--section-start=.mmcu=0x910000" # special flags for simavr
test10.elf: MCU = atmega1284p
test10.elf: F_OSC = 20000000
test10.elf: test10.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ test10.o simavrconsole.o

-include ../Makefile.include

