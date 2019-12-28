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

# fuses: SUT auf 64ms
# ~/Projekte/updi/pyupdi/pyupdi.py -d mega4808 -fs 6:0x07 -c /dev/ttyUSB1

# fuses: 20MHz
# ~/Projekte/updi/pyupdi/pyupdi.py -d mega4808 -fs 2:0x02 -c /dev/ttyUSB1

# flashing
# ~/Projekte/updi/pyupdi/pyupdi.py -d mega4808 -c /dev/ttyUSB1  -f test20.hex

MCU = atmega4809 # geht auch 4808
F_OSC = 20000000

subdirs =

targets += test20.elf test20.hex
targets += test21.elf test21.hex
targets += test22.elf test22.hex
targets += test23.elf test23.hex
targets += test30.elf test30.hex
targets += test40.elf test40.hex
targets += test41.elf test41.hex
targets += test50.elf test50.hex
targets += test51.elf test51.hex
targets += test52.elf test52.hex
targets += test53.elf test53.hex
targets += test54.elf test54.hex
targets += test55.elf test55.hex
targets += test56.elf test56.hex
targets += test57.elf test57.hex 
targets += test58.elf test58.hex test58.S
targets += test59.elf test59.hex test59.S
targets += test60.elf test60.hex test60.S
targets += test61.elf test61.hex test61.S

targets += test99.elf test99.hex

-include ../../../Makefile.avr0.include
