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

MCU = atmega324pb
F_OSC = 20000000

LOADLIBES = simavrconsole.o

targets += simavrconsole.o

targets += blink.elf
targets += console.elf
targets += delay.elf
targets += ports.elf
targets += isr01.elf isr01.s
targets += isr02.elf isr02.s
targets += isr_test01.elf isr_test01.s
targets += eventhandler.elf
targets += events.elf
targets += interrupts.elf
targets += pinchange.elf
#targets += softusart.elf
targets += constantrate.elf constantrate.s 
#targets += menu.elf menu.s
targets += menu_s.elf menu_s.s
#targets += ledbuttons.elf ledbuttons.s 
targets += pinset.elf pinset.s pinset.S

-include ../../Makefile.include

