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

CPU = cortex-m3
MCU = stm32f103c8

F_OSC = 8000000

CPPFLAGS += -DNDEBUG
CPPFLAGS += -I../../include/mcu/stm32

LDSCRIPT = stm32f103c8.ld
PTYPE = STM32F10X_MD_VL

LOADLIBES = startup_stm32f10x.o system_stm32f10x.o

targets += shtml
targets +=  bm01a.s bm01a.elf
targets +=  bm01b.s bm01b.elf

bm01a.elf: bm01a.o startup_stm32f10x.o system_stm32f10x.o

bm01b.elf: bm01b.o startup_stm32f10x.o system_stm32f10x.o

-include ../../Makefile.arm.include

