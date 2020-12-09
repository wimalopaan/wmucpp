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

# ~/Projekte/updi/pyupdi/pyupdi.py -d tiny1614 -c /dev/ttyUSB1  -f test00.hex
# ~/.local/bin/pyupdi -d tiny1614 -c /dev/ttyUSB1  -f test00.hex

MCU = attiny1614
F_OSC = 20000000

subdirs =

#targets += test00.elf test00.hex

targets += a1g.hex
targets += a4uni.hex
targets += a4t.hex
targets += a3t2r.hex
targets += a2t2c.hex
targets += a2c2r.hex
targets += a2c2r1g.hex
targets += a2t2r1g.hex
targets += a1t1u1r1g.hex
targets += b10c5.hex
targets += b11c4.hex
targets += ibus2multi.hex
targets += servoswitch.hex
targets += servo360.hex
targets += sbus_test.hex
targets += sport_test.hex

-include ../../Makefile.avr0.include
