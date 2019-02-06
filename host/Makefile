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

subdirs += hott

#test23a: CXXFLAGS = -g -std=c++17 -Wall -Wextra -fPIC -I../include 
#test23a: CXX = clang++
#test02a: CXXFLAGS = -g -std=c++17 -Wall -Wextra -fPIC -I../include 
#test02a: CXX = clang++
#test04: CXXFLAGS = -g -std=c++17 -Wall -Wextra -fPIC -I../include 
#test04: CXX = clang++

targets += test01 test02 test02a test03 test04 test04b # test05
targets += test06
targets += test07
targets += test10
targets += test11
targets += test12
targets += test13
targets += test15
targets += test20
targets += test21
targets += test22
targets += test23
targets += test23a
#targets += test24
#targets += test25
#targets += test26
targets += test30
targets += test35
targets += test36
targets += test37
targets += test38
targets += test40
targets += test50
targets += test51
targets += test60
targets += test70
targets += test80
targets += test90
targets += test91
targets += test92
targets += test93
targets += test98
targets += test99
targets += test100

-include ../Makefile.include

#CXX = /usr/bin/g++
CXX = g++
#CXX = clang++
CC = g++
LDFLAGS = -lpthread
CXXFLAGS = -g -std=c++2a -fconcepts -ftemplate-depth=2048
CXXFLAGS += -Wall -Wextra -fPIC
CXXFLAGS += -I../include0

MCUFLAGS =
