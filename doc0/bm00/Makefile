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

#MCU = atmega328pb
#MCU = atmega1284p
MCU = atmega4809
#MCU = attiny1614
F_OSC = 20000000

CPPFLAGS += -DNDEBUG

#OPTFLAGS = -O3

targets += shtml

targets += bm00.S bm00.elf
targets += bm01.S bm01.elf
targets += bm02.S bm02.elf
targets += bm03.S bm03.elf
targets += bm10.s bm10.elf
targets += bm11.s bm11.elf
targets += bm12.s bm12.elf
targets += bm13.s bm13.elf
targets += bm20.S bm20.elf
targets += bm30.S bm30.elf
targets += bm31.S bm31.elf
targets += bm40.S bm40.elf
targets += bm41.S bm41.elf
targets += bm42.S bm42.elf
targets += bm43.S bm43.elf
targets += bm44.S bm44.elf

-include ../../Makefile.avr0.include

