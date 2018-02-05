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

MCU = atmega1284p
F_OSC = 20000000

subdirs =

# avr-nm -S -n -C -t d esptest.elf 

targets += main.elf
targets += owtest01.elf
targets += esptest.elf esptest.s
targets += itoatest.elf itoatest.s
targets += gpstest.elf gpstest.s
targets += spissd1306.elf spissd1306.s
targets += test01.elf test01.s
targets += test02.elf test02.s
targets += test03.elf test03.s
targets += test04.elf test04.s
targets += test05.elf test05.s

owtest01.elf: owtest01.o ow.o
	$(CC) $(LDFLAGS) -o $@ owtest01.o ow.o

-include ../../../Makefile.include

