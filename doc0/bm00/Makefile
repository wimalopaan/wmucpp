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

F_OSC = 20000000

#MCU = atmega328p
#MCU = atmega1284p
MCU = atmega4809
#MCU = attiny1614
##MCU = atxmega32e5

#bm01.s bm01.elf : MCU = atmega88p
#bm02.s bm02.elf : MCU = atmega88p
#bm01a.s bm01a.elf : MCU = atmega88p
#bm30.s bm30.elf : MCU = atmega4809
#bm43.s bm43.elf : MCU = atmega4809
#bm44.s bm44.elf : MCU = atmega4809
#bm97.d bm97.s bm97.elf : CXXINLUDES4 := -I../../../from-scratch/include

#bm03.s bm03.elf : CPPFLAGS = -xc

CPPFLAGS += -DNDEBUG
#CXXFLAGS += -fno-jump-tables

OPTFLAGS = -Os

#targets += shtml

#bm30: bm30.o bm30a.o

#targets += bm00.S bm00.elf
#targets += bm01.S bm01.elf
#targets += bm01a.S bm01a.elf
targets += bm02.S bm02.elf
#targets += bm03.S bm03.elf
#targets += bm09a.S bm09a.elf
#targets += bm09b.S bm09b.elf
targets += bm10.S bm10.elf
#targets += bm10.S bm10.elf
#targets += bm11.S bm11.elf
#targets += bm12.S bm12.elf
#targets += bm13.S bm13.elf
#targets += bm15.S bm15.elf
#targets += bm16.S bm16.elf
#targets += bm17.S bm17.elf
##targets += bm20.S bm20.elf
#targets += bm25.S bm25.elf
#targets += bm30.S bm30.elf
##targets += bm31.S bm31.elf
#targets += bm40.S bm40.elf
#targets += bm41.S bm41.elf
#targets += bm42.S bm42.elf
#targets += bm43.S bm43.elf
#targets += bm44.S bm44.elf
#targets += bm45.S bm45.elf
#targets += bm46.S bm46.elf
#targets += bm47.S bm47.elf
#targets += bm48.S bm48.elf
##targets += bm50.S bm50.elf
#targets += bm51.S bm51.elf
#targets += bm60.S bm60.elf
#targets += bm61.S bm61.elf
#targets += bm62.S bm62.elf
#targets += bm63.S bm63.elf
#targets += bm64.S bm64.elf
#targets += bm70.S bm70.elf
#targets += bm71.S bm71.elf
#targets += bm80.S bm80.elf
#targets += bm85.S bm85.elf
#targets += bm86.S bm86.elf
#targets += bm87.S bm87.elf
targets += bm90.S bm90.elf
targets += bm91.S bm91.elf
targets += bm92.S bm92.elf
#targets += bm93.S bm93.elf
#targets += bm94.S bm94.elf
#targets += bm95.S bm95.elf
#targets += bm96.S bm96.elf
#targets += bm97.S bm97.elf
#targets += bm99.S bm99.elf

-include ../../Makefile.avr0.include

