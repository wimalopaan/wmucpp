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

# ~/Projekte/updi/pyupdi/pyupdi.py -d mega4809 -c /dev/ttyUSB3  -f test00.hex

MCU = atmega4809
#MCU = avr128da48
#MCU = attiny1614
#F_OSC = 20000000
F_OSC = 32000000

subdirs =

charly.s: MCU = avr128da48
charly.hex: MCU = avr128da48
charly.elf: MCU = avr128da48

targets += charly.elf charly.S charly.hex
#targets += test00.elf test00.S test00.hex
targets += test01.elf test01.S test01.hex
targets += test02.elf test02.S test02.hex
#targets += test03.elf test03.S test03.hex
#targets += test04.elf test04.S test04.hex
#targets += test05.elf test05.S test05.hex
#targets += test06.elf test06.S test06.hex
#targets += test07.elf test07.S test07.hex
#targets += test08.elf test08.S test08.hex
#targets += test09.elf test09.S test09.hex
#targets += test10.elf test10.S test10.hex
#targets += test11.elf test11.S test11.hex
#targets += test12.elf test12.S test12.hex
#targets += test15.hex
#targets += test16.hex
#targets += test20.elf test20.S test20.hex
#targets += test21.elf test21.S test21.hex
#targets += test30.elf test30.S test30.hex
#targets += test40.elf test40.S test40.hex
#targets += test50.elf test50.S test50.hex
#targets += test60.hex
#targets += test61.hex
#targets += test70.hex
#targets += test71.hex
#targets += test80.hex
#targets += test81.hex
#targets += test82.hex
#targets += test90.elf test90.S test90.hex
#targets += test99.elf test99.S test99.hex

-include ../../Makefile.avr0.include
