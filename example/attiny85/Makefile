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

MCU = attiny85
F_OSC = 8000000

# 16 Mhz, 1k 4ms, 2,7V Brownout
# sudo avrdude -p attiny85 -P usb -c avrisp2 -U -U lfuse:w:0xc1:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m

swusart.elf: LOADLIBES = simavrconsole.o

targets += simavrconsole.o
targets +=  blink.elf
targets +=  i2cslave.elf
targets +=  i2cram.elf
targets +=  ws2812.elf
targets +=  i2cws2812.elf
targets += swusart.elf

-include ../../Makefile.include

