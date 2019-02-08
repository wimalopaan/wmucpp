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

#MCU = attiny85
MCU = atmega328p
F_OSC = 16000000

#OPTFLAGS = -O3

bm60a.elf bm60a.s: MCU = atmega328p
bm61a.elf bm61a.s: MCU = atmega328p
bm62a.elf bm62a.s: MCU = atmega328p 
bm62a.elf bm62a.s: LOADLIBES = simavrconsole.o
bm70a.elf bm70a.s: MCU = atmega328p
bm71a.elf bm71a.s: MCU = atmega328p 
bm71a.elf bm71a.s: LOADLIBES = simavrconsole.o
bm72a.elf bm72a.s: MCU = atmega328p 
bm72a.elf bm72a.s: LOADLIBES = simavrconsole.o
bm74a.elf bm74a.s: MCU = atmega328p 
bm74a.elf bm74a.s: LOADLIBES = simavrconsole.o
bm73a.elf bm73a.s: MCU = atmega328p 
bm73a.elf bm73a.s: LOADLIBES = simavrconsole.o
bm80a.elf bm80a.s: MCU = atmega328p 
bm80a.elf bm80a.s: LOADLIBES = simavrconsole.o

bm91a.elf bm91a.s: LOADLIBES = simavrconsole.o
bm96a.elf bm96a.s: LOADLIBES = simavrconsole.o

bm102a.elf bm102a.s: LOADLIBES = simavrconsole.o

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
#targets +=  bm70a.elf bm70a.s
#targets +=  bm70b.elf bm70b.s
targets +=  bm71a.elf bm71a.s
targets +=  bm72a.elf bm72a.s
#targets +=  bm73a.elf bm73a.s
targets +=  bm74a.elf bm74a.s
targets +=  bm80a.elf bm80a.s
targets +=  bm90a.elf bm90a.s
targets +=  bm90a2.elf bm90a2.s
targets +=  bm90a3.elf bm90a3.s
targets +=  bm90b.elf bm90b.s
targets +=  bm91a.elf bm91a.s
targets +=  bm92a.elf bm92a.s
targets +=  bm93a.elf bm93a.s
targets +=  bm94a.elf bm94a.s
targets +=  bm95a.elf bm95a.s
#targets +=  bm96a.elf bm96a.s bm96a.S
targets +=  bm100a.elf bm100a.s bm100a.S
targets +=  bm101a.elf bm101a.s bm101a.S
targets +=  bm102a.elf

-include ../../Makefile.include

#bm05a.elf: bm05a.o simavrconsole.o
#	$(CC) $(LDFLAGS) -o $@ bm05a.o simavrconsole.o

