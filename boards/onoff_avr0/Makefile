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

# sudo avrdude -p attiny85 -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0x62:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m

MCU = attiny412
F_OSC = 2000000

subdirs = small big
#subdirs = big

-include ../../Makefile.include
