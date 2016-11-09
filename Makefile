# -*- mode: makefile-gmake; -*-
# $Id: Makefile.include 1209 2016-07-01 07:53:57Z wimalopaan $
#
# WMuCpp - Bare Metal C++ 
# Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

all: doc example host test

.PHONY: test example clean host doc

test:
	make -C test all

host:
	make -C host all

example:
	make -C example all

doc:
	make -C doc all

clean:
	make -C doc clean
	make -C example clean
	make -C test clean
	make -C host clean

