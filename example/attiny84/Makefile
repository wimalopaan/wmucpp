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

MCU = attiny84
F_OSC = 8000000

# 8 Mhz, 6k 4ms, 2,7V Brownout
# sudo avrdude -p attiny84 -P usb -c avrisp2 -U -U lfuse:w:0xd2:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m

#LOADLIBES = simavrconsole.o

#targets += simavrconsole.o
targets +=  blink.elf
targets +=  ws2812.elf
targets +=  spi.elf
targets +=  console.elf
targets +=  spiws2812.elf
targets +=  i2cslave.elf
targets +=  i2cram.elf

-include ../../Makefile.include

