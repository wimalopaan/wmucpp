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

targets += uart01
targets += test01
targets += test02
targets += test03

-include ../../Makefile.include

CXX = /usr/bin/g++
#CXX = clang++
CC = g++
LDFLAGS = -lpthread
CXXFLAGS = -g -std=c++17 -fconcepts
CXXFLAGS += -Wall -Wextra -fPIC
CXXFLAGS += -I../include 
