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

# 1. Fuses (16MHz, BootLoader)
# sudo avrdude -p atmega328pb -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd4:m -U efuse:w:0xff:m
# 2. Bootloader flached -> booloader/optiboot.txt
# 
# sudo avrdude -p atmega328pb -c arduino -b 9600 -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P /dev/ttyUSB2 -U flash:w:test00.elf

# Optiboot 19200 Uart0
# sudo avrdude -p atmega1284p -P usb -c avrisp2 -V -U flash:w:optiboot_flash_atmega1284p_UART0_19200_20000000L.hex

# sudo avrdude -p atmega1284p -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m

# ~/.local/bin/pymcuprog write --erase -d avr128da32 -t uart -u /dev/ttyUSB0 -f test10.hex


#MCU = atmega1284p
MCU = avr128da32
#F_OSC = 20000000
F_OSC = 32000000

#OPTFLAGS = -O3

#CXXFLAGS += -ffreestanding


subdirs =

#targets += test00.elf test00.S
#targets += test01.elf
#targets += test02.elf
#targets += test03.elf
#targets += test04.elf
targets += da00.hex
targets += da01.hex
targets += da02.hex
targets += si5351.hex
targets += fsk00.hex
#targets += mswitch01.hex

-include ../../Makefile.avr0.include
