# -*- mode: makefile-gmake; -*-
#
# WMuCpp - Bare Metal C++ 
# Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-flash --reset --connect-under-reset write cc01.bin  0x8000000
# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace --trace=24000000     

CPU = cortex-m0plus
#FPU = -mfpu=softvfp
#FLOAT-ABI = -mfloat-abi=soft
OPTFLAGS = -g -Os

CPPFLAGS += -I../../include_stm32
CPPFLAGS += -I../../../STM32CubeG0/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../STM32CubeG0/Drivers/CMSIS/Device/ST/STM32G0xx/Include
CPPFLAGS += -I../../../STM32CubeG0/Drivers/STM32G0xx_HAL_Driver/Inc
CPPFLAGS += -DSTM32G0B1xx

LDSCRIPT = link_g0b1.ld
LOADLIBES += startup_g0b1.o system_g0.o

targets += startup_g0b1.o
targets += system_g0.o

#targets += pult_01.elf pult_01.bin
targets += desk_01.elf desk_01.bin

-include ../../Makefile.arm.include
