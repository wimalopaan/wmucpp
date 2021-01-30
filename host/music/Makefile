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

#targets += test01

-include ../../Makefile.include

CXX = /usr/local/bin/g++
CC = /usr/local/bin/gcc
#LDFLAGS = -lasan -lubsan -lpthread -lstdc++ 
LDFLAGS = -lpthread -lstdc++ 
#CXXFLAGS = -g -std=c++2a -fconcepts -ftemplate-depth=2048 -fno-threadsafe-statics # -fsanitize=address -fsanitize=undefined 
CXXFLAGS = -g -std=c++2a -fconcepts 
CXXFLAGS += -Wall -Wextra -O3
CXXFLAGS += -I../../include0
MCUFLAGS =
