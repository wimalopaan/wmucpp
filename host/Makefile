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

#subdirs += hott music tools
subdirs += sdr

#test23a: CXXFLAGS = -g -std=c++17 -Wall -Wextra -fPIC -I../include 
#test23a: CXX = clang++
#test02a: CXXFLAGS = -g -std=c++17 -Wall -Wextra -fPIC -I../include 
#test02a: CXX = clang++
#test04: CXXFLAGS = -g -std=c++17 -Wall -Wextra -fPIC -I../include 

#targets += rswitch
#targets += rdiv

#test23: test23.cc test23a.cc

#targets += test01 
#targets += test02 
#targets += test04
##targets += test04b
#targets += test06
##targets += test07
#targets += test08
###targets += test10
##targets += test11
##targets += test12
##targets += test13
##targets += test15
##targets += test20
##targets += test21
##targets += test22
##targets += test23
##targets += test23a
###targets += test24
###targets += test25
###targets += test26
##targets += test30
##targets += test35
##targets += test36
##targets += test37
##targets += test38
##targets += test40
##targets += test50
##targets += test51
#targets += test60
##targets += test70
##targets += test71
##targets += test72
##targets += test73
#targets += test74
#targets += test80
#targets += test85
##targets += test86
##targets += test87
#targets += test88
#targets += test90
#targets += test91
#targets += test92
##targets += test93
#targets += test94
##targets += test98
#targets += test99
#targets += test99b
#targets += test100
targets += test200
##targets += test201
#targets += test210
#targets += test300
targets += test400
targets += test401
#targets += p00a

#test400: modA.o test400.o
#	$(LINK.cc) $^

#p00a: p00a.o p00b.o

#p00a.o: p00a.c p00.h

#p00b.o: p00b.c p00.h

-include ../Makefile.include

#CXX = /usr/bin/g++
CXX = /usr/local/bin/g++
CC = /usr/bin/gcc
#CXX = clang++
#CC = /usr/local/bin/gcc
#LDFLAGS = -lasan -lubsan -lpthread -lstdc++ 
#LDFLAGS = -lpthread -lstdc++ 
CXXFLAGS = -g -std=c++23 -fconcepts -fmodules-ts -ftemplate-depth=2048 -fno-threadsafe-statics  -fsanitize=address -fsanitize=undefined 
CXXFLAGS += -O3
CXXFLAGS += -Wall -Wextra
CXXFLAGS += -I../include0/etl
CFLAGS = -Wall -Wextra -g -std=gnu2x
CFLAGS += -O3
MCUFLAGS =

%.o: %.cxx
	$(CXX) -std=c++23 -c -fmodules-ts -x c++ $< -o $@ 
