# -*- mode: makefile-gmake; -*-
#
# WMuCpp - Bare Metal C++
# Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

# ~/Projekte/updi/pyupdi/pyupdi.py -d tiny1614 -c /dev/ttyUSB1  -f test00.hex

MCU = attiny1614
F_OSC = 20000000

# tiny1624 needed adapted linker script avrxmega3.xn
#auto8d.hex: MCU = attiny1624

subdirs =

# digital (OpenTx)
# multi8d (ibus/sbus)

# Graupner Multikanal MK4
# multi8mk4  (ppm)

# multi8hott (über telemetrie / ascii menu)

# Langsamer Modus - wie Momo-Sauerland / Engel-Modelltechnik
# multi8man     (ppm)
# multi8manhott (sumd)

#targets += test00.elf test00.hex test00.S
#targets += test01.elf test01.hex
#targets += test10.elf test10.hex
#targets += multi8d.hex
#targets += multi8dsbus.hex # not needed anymore
#targets += multi8mk4.hex
#targets += multi8hott.hex
#targets += multi8man.hex
#targets += auto8d.hex
targets += crsf01.hex
targets += sbus01.hex

-include ../../Makefile.avr0.include
