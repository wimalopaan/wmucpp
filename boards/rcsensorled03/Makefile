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

# sudo avrdude -p atmega328pb -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m -U efuse:w:0xf7:m

# EEProm Inhalt nicht löschen
# sudo avrdude -p atmega328pb -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xe0:m -U hfuse:w:0xd1:m -U efuse:w:0xf7:m

MCU = atmega328pb
F_OSC = 20000000

subdirs =

#targets += test01.s test01.elf
#targets += test02.s test02.elf
targets += test03.s test03.elf test03.S
#targets += test04.s test04.elf test04.S
#targets += test05.s test05.elf test05.S
#targets += test06.s test06.elf test06.S
#targets += test06a.s test06a.elf test06a.S
#targets += test06b.s test06b.elf test06b.S
#targets += test07.s test07.elf test07.S
#targets += test08.s test08.elf test08.S
#targets += test09.s test09.elf test09.S
#targets += test10.s test10.elf
#targets += test11.s test11.elf test11.S
#targets += test12.s test12.elf test12.S
#targets += test13.s test13.elf test13.S
#targets += test14.s test14.elf test14.S
#targets += test15.s test15.elf test15.S
#targets += test16.s test16.elf test16.S
#targets += test17.s test17.elf test17.S
#targets += test18.s test18.elf test18.S
#targets += test19.s test19.elf test19.S
#targets += test20.s test20.elf
#targets += test21.s test21.elf
#targets += test22.s test22.elf
#targets += test23.s test23.elf
targets += test24.s test24.elf
#targets += oledtest.elf 
#targets += test50.s test50.elf test50.S
#targets += test51.s test51.elf test51.S

-include ../../Makefile.include
