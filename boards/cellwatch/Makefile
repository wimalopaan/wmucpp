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


# ~/.local/bin/pyupdi -c /dev/ttyUSB0 -d tiny412 -fs 2:0x01 -f base04.hex 
# ~/.local/bin/pyupdi -c /dev/ttyUSB0 -d tiny412 -f cell04.hex 

MCU = attiny412
F_OSC = 20000000

#cell04.hex : F_OSC = 10000000
cell04.hex : F_OSC = 5000000
base04.hex : F_OSC = 16000000

subdirs =

#targets += test00.hex
#targets += test01.hex
#targets += test02.hex
#targets += test03.hex
#targets += test04.hex
#targets += base01.hex
#targets += cell01.hex
#targets += base02.hex
#targets += cell03.hex
#targets += base03.hex
targets += cell04.hex
targets += base04.hex

-include ../../Makefile.avr0.include
