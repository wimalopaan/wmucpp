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
F_OSC = 12000000
#MCU = atmega1284p
#F_OSC = 20000000

AS = avr-as -mmcu=$(MCU)

bm01a.elf : LOADLIBES = simavrconsole.o
bm02a.elf : LOADLIBES = simavrconsole.o
bm03a.elf : LOADLIBES = simavrconsole.o
bm04a.elf : LOADLIBES = simavrconsole.o
bm05a.elf : LOADLIBES = simavrconsole.o
bm06a.elf : LOADLIBES = simavrconsole.o
bm10a.elf : LOADLIBES = simavrconsole.o
bm30a.elf : LOADLIBES = simavrconsole.o ascale.o
bm30b.elf : LOADLIBES = simavrconsole.o
bm50a.elf : LOADLIBES = simavrconsole.o
bm60a.elf : LOADLIBES = simavrconsole.o
bm51a.elf : LOADLIBES = simavrconsole.o
bm55a.elf : LOADLIBES = simavrconsole.o
#bm37a.elf:  LOADLIBES = ascale.o

targets += simavrconsole.o ascale.o
targets +=  bm01a.s bm01a.elf 
targets +=  bm02a.s bm02a.elf 
targets +=  bm03a.s bm03a.elf 
targets +=  bm04a.s bm04a.elf 
targets +=  bm05a.s bm05a.elf 
targets +=  bm06a.s bm06a.elf 
targets +=  bm10a.s bm10a.elf 
targets +=  bm20a.s bm20a.elf 
targets +=  bm30a.s bm30a.elf 
targets +=  bm30b.s bm30b.elf 
targets +=  bm31a.s bm31a.elf 
targets +=  bm32a.s bm32a.elf 
targets +=  bm33a.s bm33a.elf 
targets +=  bm34a.s bm34a.elf 
targets +=  bm34b.s bm34b.elf 
targets +=  bm34c.s bm34c.elf 
targets +=  bm35a.s bm34a.elf 
targets +=  bm36a.s bm36a.elf 
targets +=  bm37a.s bm37a.elf 
targets +=  bm38a.s bm38a.elf 
targets +=  bm50a.s bm50a.elf 
targets +=  bm51a.s bm51a.elf 
targets +=  bm52a.s bm52a.elf 
targets +=  bm53a.s bm53a.elf 
targets +=  bm54a.s bm54a.elf 
targets +=  bm55a.s bm55a.elf 
targets +=  bm60a.s bm60a.elf 
targets +=  bm61a.s bm61a.elf 
targets +=  bm70b.s bm70b.elf 

-include ../../Makefile.include

bm37a.elf: bm37a.o ascale.o
	$(CC) $(LDFLAGS) -o $@ bm37a.o ascale.o


