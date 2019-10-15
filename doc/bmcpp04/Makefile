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
F_OSC = 20000000

bm30a.s: OPTFLAGS = -O0
bm30b.s: OPTFLAGS = -O0

bm40a.elf: LOADLIBES=simavrconsole.o

targets +=  bm01a.s bm01a.elf bm01a.size
targets +=  bm01b.s bm01b.elf bm01b.size
targets +=  bm02a.s bm02a.elf bm02a.size
targets +=  bm02b.s bm02b.elf bm02b.size
targets +=  bm03a.s bm03a.elf bm03a.size
targets +=  bm03b.s bm03b.elf bm03b.size
targets +=  bm04a.s bm04a.elf bm04a.size
targets +=  bm05a.s bm05a.elf bm05a.size
targets +=  bm05aa.s bm05aa.elf bm05aa.size
targets +=  bm05b.s bm05b.elf bm05b.size
targets +=  bm05bb.s bm05bb.elf bm05bb.size
targets +=  bm06a.s bm06a.elf bm06a.size
targets +=  bm07a.s bm07a.elf bm07a.size
targets +=  bm10a.s bm10a.elf
targets +=  bm11a.s bm11a.elf
targets +=  bm20a.s bm20a.elf bm20a.S
targets +=  bm30b.s bm30b.elf
targets +=  bm30a.s bm30a.elf
targets +=  bm40a.s bm40a.elf
targets +=  bm40b.s bm40b.elf
targets +=  bm41a.s bm41a.elf
targets +=  bm42a.s bm42a.elf
targets +=  bm50a.s bm50a.elf

CPPFLAGS += -DNDEBUG

-include ../../Makefile.include

bm02a.elf: bm02a.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ bm02a.o simavrconsole.o

bm02b.elf: bm02b.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ bm02b.o simavrconsole.o
