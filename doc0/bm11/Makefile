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

# ~/Projekte/updi/pyupdi/pyupdi.py -d tiny1614 -c /dev/ttyUSB1  -f bm01.hex

F_OSC = 20000000

#MCU = atmega328pb
#MCU = atmega1284p
#MCU = atmega4809
MCU = attiny1614
#MCU = atxmega32e5

CPPFLAGS += -DNDEBUG
CXXINLUDES4 = -I /home/lmeier/Projekte/cnl/cnl/include

OPTFLAGS = -O3

targets += shtml

targets += bm00.S bm00.elf bm00.hex
targets += bm01.S bm01.elf bm01.hex
targets += bm02.S bm02.elf bm02.hex
targets += bm10.S bm10.elf bm10.hex
targets += bm20.S bm20.elf bm20.hex
#targets += bm02.S bm02.elf

-include ../../Makefile.avr0.include

