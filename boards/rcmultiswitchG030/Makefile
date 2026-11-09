# -*- mode: makefile-gmake; -*-
#
# WMuCpp - Bare Metal C++ 
# Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

HW ?= HW_MSW12
TOOLCHAIN ?= gcc

ifneq ($(words $(HW)),1)
$(error Select exactly one HW)
endif

ifeq ($(HW),HW_MSW10)
DEFAULT_MCU = G030
else ifneq ($(filter HW_MSW11 HW_NUCLEO HW_WEACT,$(HW)),)
DEFAULT_MCU = G031
else ifneq ($(filter HW_MSW12 HW_WMG0B1,$(HW)),)
DEFAULT_MCU = G0B1
else
$(error Unsupported HW '$(HW)'; expected HW_MSW10, HW_MSW11, HW_MSW12, HW_NUCLEO, HW_WEACT or HW_WMG0B1)
endif

MCU ?= $(DEFAULT_MCU)
HW_DEFINES = $(HW)

ifeq ($(HW),HW_MSW12)
ifeq ($(MCU),G051)
HW_DEFINES += HW_MSW12_G051
else ifneq ($(MCU),G0B1)
$(error Unsupported MCU '$(MCU)' for $(HW); expected G0B1 or G051)
endif
else ifneq ($(MCU),$(DEFAULT_MCU))
$(error Unsupported MCU '$(MCU)' for $(HW); expected $(DEFAULT_MCU))
endif

ifeq ($(MCU),G030)
MCU_DEFINE = STM32G030xx
STARTUP = startup_g030
LDSCRIPT = link_g030.ld
else ifeq ($(MCU),G031)
MCU_DEFINE = STM32G031xx
STARTUP = startup_g031
LDSCRIPT = link_g031.ld
else ifeq ($(MCU),G051)
MCU_DEFINE = STM32G051xx
STARTUP = startup_g051
LDSCRIPT = link_g051.ld
else ifeq ($(MCU),G0B1)
MCU_DEFINE = STM32G0B1xx
STARTUP = startup_g0b1
LDSCRIPT = link_g0b1.ld
endif

ifeq ($(TOOLCHAIN),gcc)
OPTFLAGS ?= -Os -flto
else ifeq ($(TOOLCHAIN),clang)
OPTFLAGS ?= -Oz -flto
else
$(error Unsupported TOOLCHAIN '$(TOOLCHAIN)'; use gcc or clang)
endif

export LC_ALL = C

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CUBE_DIR ?= ../../../STM32CubeG0
MCUFLAGS = -mthumb -mcpu=cortex-m0plus
CPPFLAGS += -I. -I../../include_stm32
CPPFLAGS += -I$(CUBE_DIR)/Drivers/CMSIS/Core/Include
CPPFLAGS += -I$(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32G0xx/Include
CPPFLAGS += -I$(CUBE_DIR)/Drivers/STM32G0xx_HAL_Driver/Inc
CPPFLAGS += -D$(MCU_DEFINE) $(addprefix -D,$(HW_DEFINES))

CXXFLAGS += $(OPTFLAGS) -std=c++26
CXXFLAGS += -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics
CXXFLAGS += -funsigned-char -funsigned-bitfields -fshort-enums
CXXFLAGS += -ffunction-sections -fdata-sections -ftemplate-depth=2048
CXXFLAGS += -fstrict-aliasing -Wstrict-aliasing=1 -Wall -Wextra $(MCUFLAGS)

LDFLAGS += -Wl,--gc-sections $(MCUFLAGS) -T$(LDSCRIPT)

ifeq ($(TOOLCHAIN),clang)
CXX = clang++
CXXFLAGS := $(filter-out -funsigned-bitfields -Wstrict-aliasing=1,$(CXXFLAGS))
GCC_SYSROOT := $(shell arm-none-eabi-g++ -print-sysroot)
GCC_VERSION := $(shell arm-none-eabi-g++ -dumpfullversion)
GCC_MULTILIB := $(shell arm-none-eabi-g++ $(MCUFLAGS) -print-multi-directory)
CLANG_TARGET_FLAGS = --target=arm-none-eabi --gcc-toolchain=$(dir $(GCC_SYSROOT))
CLANG_TARGET_FLAGS += -Wno-multilib-not-found
CXXFLAGS += $(CLANG_TARGET_FLAGS) -stdlib=libstdc++
CXXFLAGS += -isystem $(GCC_SYSROOT)/include/c++/$(GCC_VERSION)/arm-none-eabi/$(GCC_MULTILIB)
GCC_LIBDIR := $(dir $(shell arm-none-eabi-g++ $(MCUFLAGS) -print-libgcc-file-name))
NEWLIB_LIBDIR := $(dir $(shell arm-none-eabi-g++ $(MCUFLAGS) -print-file-name=libc_nano.a))
else
LDFLAGS += -Wl,--no-warn-rwx-segments -specs=nano.specs
endif

BUILD_DIR = build/$(HW)/$(MCU)/$(TOOLCHAIN)
SOURCES = msw30.cc system_g0.cc
OBJECTS = $(patsubst %.cc,$(BUILD_DIR)/%.o,$(SOURCES)) $(BUILD_DIR)/$(STARTUP).o
DEPS = $(patsubst %.cc,$(BUILD_DIR)/%.d,$(SOURCES))
ELF = $(BUILD_DIR)/msw30.elf
BIN = $(BUILD_DIR)/msw30.bin

.DEFAULT_GOAL := all
.DELETE_ON_ERROR:
.PHONY: all clean

all: $(BIN)

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/%.o: %.cc Makefile | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/$(STARTUP).o: $(STARTUP).s Makefile | $(BUILD_DIR)
	$(CC) $(MCUFLAGS) -c $< -o $@

$(ELF): $(OBJECTS) $(LDSCRIPT) Makefile
ifeq ($(TOOLCHAIN),clang)
	$(CXX) $(CLANG_TARGET_FLAGS) $(OPTFLAGS) $(LDFLAGS) -fuse-ld=lld -nostdlib \
		$(GCC_LIBDIR)crti.o $(GCC_LIBDIR)crtbegin.o $(NEWLIB_LIBDIR)crt0.o \
		$(OBJECTS) -L$(GCC_LIBDIR) -L$(NEWLIB_LIBDIR) \
		-Wl,--start-group -lstdc++_nano -lm -lgcc -lc_nano $(LOADLIBES) -Wl,--end-group \
		$(GCC_LIBDIR)crtend.o $(GCC_LIBDIR)crtn.o -o $@
else
	$(CXX) $(OBJECTS) $(OPTFLAGS) $(LDFLAGS) $(LOADLIBES) -o $@
endif

$(BIN): $(ELF)
	$(OBJCOPY) -O binary -S $< $@
	$(SIZE) $<

clean:
	$(RM) -- $(OBJECTS) $(DEPS) $(ELF) $(BIN)

-include $(DEPS)
