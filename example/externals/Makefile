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

MCU = atmega1284p
F_OSC = 20000000

#LOADLIBES = simavrconsole.o

#targets += simavrconsole.o

targets += buttonmacro.s buttonmacro.elf
targets += button.s button.elf
targets += ds18b20.elf
targets += ds18b20async.elf
targets += onewire.elf
targets += ws2812.elf
targets += analog.elf
targets += analogevent.elf
targets += akku.elf
targets += ds1307.elf
targets += ds1307async.elf
targets += i2cattiny.elf
targets += i2cram.elf
targets += i2cramasync.elf
targets += tle5205.elf
targets += rpm.elf
targets += rpmsofttimer.elf
targets += pinchange.elf

onewire.elf: onewire.o ow.o
	$(CC) $(LDFLAGS) -o $@ onewire.o ow.o


-include ../../Makefile.include

