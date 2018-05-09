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
#MCU = atmega1284p
F_OSC = 20000000

bm01a.elf : LOADLIBES = simavrconsole.o
bm02a.elf : LOADLIBES = simavrconsole.o
bm03a.elf : LOADLIBES = simavrconsole.o
bm10a.elf bm10b.elf: LOADLIBES = simavrconsole.o
bm11a.elf : LOADLIBES = simavrconsole.o

targets += simavrconsole.o
targets +=  bm01a.s bm01a.elf 
targets +=  bm02a.s bm02a.elf 
targets +=  bm02x.s bm02x.elf 
targets +=  bm02y.s bm02y.elf 
targets +=  bm02z.s bm02z.elf 
targets +=  bm03a.s bm03a.elf 
targets +=  bm04a.s bm04a.elf 
targets +=  bm10a.s bm10a.elf 
targets +=  bm10b.s bm10b.elf 
targets +=  bm11a.s bm11a.elf 
targets +=  bm20a.s bm20a.elf bm20a.lss 
targets +=  bm20b.s bm20b.elf bm20b.lss
targets +=  bm21a.s bm21a.elf bm21a.lss 
targets +=  bm21b.s bm21b.elf bm21b.lss
targets +=  bm30a.s bm30a.elf 

-include ../../Makefile.include

#bm01a.elf: bm01a.o simavrconsole.o
#	$(CC) $(LDFLAGS) -o $@ bm01a.o simavrconsole.o


