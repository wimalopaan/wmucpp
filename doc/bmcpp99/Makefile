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

MCU = atmega328p
F_OSC = 20000000

bm50a.elf: LOADLIBES=simavrconsole.o

targets = shtml

targets +=  bm01a.s bm01a.elf bm01a.size bm01a.cc.extract bm01a.s.axtract
targets +=  bm01b.s bm01b.elf bm01b.size
targets +=  bm02a.s bm02a.elf bm02a.size
targets +=  bm10a.elf
targets +=  bm20a.elf bm20a.s
targets +=  bm21a.elf bm21a.s
targets +=  bm30a.elf bm30a.s
targets +=  bm30b.elf bm30b.s
targets +=  bm05a.elf bm05a.s
#targets +=  bm40a.elf bm40a.s
targets +=  bm41a.elf bm41a.s
targets +=  bm42a.elf bm42a.s
targets +=  bm50a.elf bm50a.s
targets +=  bm99a.elf bm99a.s
targets += sort.h.extract

-include ../../Makefile.include

bm01a.elf: bm01a.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ bm01a.o simavrconsole.o

bm02a.elf: bm02a.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ bm02a.o simavrconsole.o

bm01b.elf: bm01b.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ bm01b.o simavrconsole.o

bm05a.elf: bm05a.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ bm05a.o simavrconsole.o

