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

CPU = cortex-m0plus
MCU = __SAMD21G18A__
PTYPE = $(MCU)

F_OSC = 8000000

#CPPFLAGS += -DNDEBUG
CPPFLAGS += -DDONT_USE_CMSIS_INIT

CPPFLAGS += -I../../include/mcu
CPPFLAGS += -I$(HOME)/Software/cmsis/Core/Include
CPPFLAGS += -I$(HOME)/Software/asf/sam0
CPPFLAGS += -I$(HOME)/Software/asf/sam0/utils/cmsis/samd21/include

#CPPFLAGS += -I$(HOME)/Software/asf/sam0/utils
#CPPFLAGS += -I$(HOME)/Software/asf/sam0/utils/preprocessor
#CPPFLAGS += -I$(HOME)/Software/asf/sam0/utils/header_files
#CPPFLAGS += -I$(HOME)/Software/asf/sam0/drivers/system/clock
#CPPFLAGS += -I$(HOME)/Software/asf/sam0/drivers/system/clock/clock_samd21_r21_da_ha1
#CPPFLAGS += -I$(HOME)/Software/asf/sam0/drivers/system/pinmux
#CPPFLAGS += -I$(HOME)/Software/asf/sam0/drivers/system/power/power_sam_d_r_h
#CPPFLAGS += -I$(HOME)/Software/asf/sam0/drivers/system/reset/reset_sam_d_r_h
#CPPFLAGS += -I$(HOME)/Software/asf/common/utils

CXXFLAGS += -Wno-expansion-to-defined

LDSCRIPT = samd21g18a_flash.ld
LDFLAGS += -specs=nano.specs

LOADLIBES = startup_samd21.o 

targets += shtml
targets +=  bm01a.s bm01a.elf
targets +=  bm02a.s bm02a.elf

bm01a.elf: bm01a.o startup_samd21.o

-include ../../Makefile.arm.include

