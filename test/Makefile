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

MCU = atmega328p
F_OSC = 20000000

targets += simavr.elf # host

CPPFLAGS += -DNDEBUG

host: CXX = /usr/local/bin/g++
#host: CXX = clang++
host: CC = g++
host: LDFLAGS =
host: CPPFLAGS = -DF_CPU=1000000
host: CXXFLAGS = -g -std=c++17 -fconcepts 
host: CXXFLAGS += -Wall -Wextra
host: CXXFLAGS += -I ../include
host: CXXFLAGS += -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics  -funsigned-char -funsigned-bitfields -fshort-enums
host: MCUFLAGS =


simavr.elf: simavr.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ simavr.o simavrconsole.o

-include ../Makefile.include
