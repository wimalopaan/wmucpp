# -*- mode: makefile-gmake; -*-
#
# WMuCpp - Bare Metal C++ 
# Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

MCU = attiny85
#MCU = atmega328p
F_OSC = 8000000


bm60a.elf bm60a.s: MCU = atmega328p
bm61a.elf bm61a.s: MCU = atmega328p
bm62a.elf bm62a.s: MCU = atmega328p 
bm62a.elf bm62a.s: LOADLIBES = simavrconsole.o
bm70a.elf bm70a.s: MCU = atmega328p
bm71a.elf bm71a.s: MCU = atmega328p 
bm71a.elf bm71a.s: LOADLIBES = simavrconsole.o
bm72a.elf bm72a.s: MCU = atmega328p 
bm72a.elf bm72a.s: LOADLIBES = simavrconsole.o

simavrconsole.o: MCU = atmega328p 

targets += simavrconsole.o
targets +=  bm05a.elf bm05a.s
targets +=  bm01a.elf bm01a.s
targets +=  bm02a.elf bm02a.s
targets +=  bm10a.elf bm10a.s
targets +=  bm10b.elf bm10b.s
targets +=  bm20b.elf bm20b.s
targets +=  bm30a.elf bm30a.s bm30a.s.axtract
targets +=  bm30b.elf bm30b.s
targets +=  bm50a.elf bm50a.s
targets +=  bm51a.elf bm51a.s
targets +=  bm60a.elf bm60a.s
targets +=  bm61a.elf bm61a.s
targets +=  bm62a.elf bm62a.s
targets +=  bm70a.elf bm70a.s
targets +=  bm71a.elf bm71a.s
targets +=  bm72a.elf bm72a.s

-include ../../Makefile.include

#bm05a.elf: bm05a.o simavrconsole.o
#	$(CC) $(LDFLAGS) -o $@ bm05a.o simavrconsole.o

