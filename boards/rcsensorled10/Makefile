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

# crtatmega324pb.o / libatmega324pb.a / specs-atmega324pb aus den Atmel-Packs
#
# Signatur in lokale avrdude.conf eingetragen

# sudo avrdude -p atmega324pb -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m -U efuse:w:0xf7:m

# EEProm Inhalt nicht löschen
# sudo avrdude -p atmega324pb -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xe0:m -U hfuse:w:0xd1:m -U efuse:w:0xf7:m

MCU = atmega324pb
F_OSC = 20000000

subdirs =

#targets += test01.s test01.elf
#targets += test02.s test02.elf
#targets += test03.s test03.elf
#targets += test04.s test04.elf
#targets += test05.s test05.elf
#targets += test06.s test06.elf
#targets += test07.s test07.elf
#targets += test08.s test08.elf
#targets += test10.s test10.elf
#targets += test11.s test11.elf
targets += test12.s test12.elf
targets += test13.s test13.elf
#targets += test14.s test14.elf
#targets += test15.s test15.elf
#targets += test16.s test16.elf
targets += test24.s test24.elf

-include ../../Makefile.include

CXXFLAGS+=-mno-gas-isr-prologues # prevent problem mit __gcc_isr

#CXXFLAGS += -Wno-deprecated-declarations
