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

# st-flash --reset --connect-under-reset write relay_01.bin 0x8000000

BOARD ?= WEACT
TOOLCHAIN ?= gcc

ifeq ($(TOOLCHAIN),gcc)
OPTFLAGS ?= -Os
else ifeq ($(TOOLCHAIN),clang)
OPTFLAGS ?= -Oz -flto
else
$(error Unsupported TOOLCHAIN '$(TOOLCHAIN)'; use gcc or clang)
endif

ifeq ($(BOARD),WEACT)
CPU = cortex-m0plus
FAMILY = G0
MCU = STM32G031xx
BOARD_DEFINE = USE_WEACT
STARTUP = startup_g031
SYSTEM = system_g0
LDSCRIPT = link_g031.ld
else ifeq ($(BOARD),WMG0B1)
CPU = cortex-m0plus
FAMILY = G0
MCU = STM32G0B1xx
BOARD_DEFINE = USE_WMG0B1
STARTUP = startup_g0b1
SYSTEM = system_g0
LDSCRIPT = link_g0b1.ld
else ifeq ($(BOARD),NUCLEO431)
CPU = cortex-m4
FAMILY = G4
MCU = STM32G431xx
BOARD_DEFINE = USE_NUCLEO_431
STARTUP = startup_g431
SYSTEM = system_g4
LDSCRIPT = link_g431.ld
FPU = -mfpu=fpv4-sp-d16 -mfloat-abi=hard
else
$(error Unsupported BOARD '$(BOARD)'; expected WEACT, WMG0B1 or NUCLEO431)
endif

export LC_ALL = C

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CUBE_DIR = ../../../STM32Cube$(FAMILY)
MCUFLAGS = -mthumb -mcpu=$(CPU) $(FPU)

CPPFLAGS += -I. -I../../include_stm32
CPPFLAGS += -I$(CUBE_DIR)/Drivers/CMSIS/Core/Include
CPPFLAGS += -I$(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32$(FAMILY)xx/Include
CPPFLAGS += -I$(CUBE_DIR)/Drivers/STM32$(FAMILY)xx_HAL_Driver/Inc
CPPFLAGS += -D$(MCU) -D$(BOARD_DEFINE)
ifeq ($(FAMILY),G4)
CPPFLAGS += -I$(CUBE_DIR)/Drivers/CMSIS/DSP/Include
CPPFLAGS += -I$(CUBE_DIR)/Drivers/CMSIS/DSP/Source -ICMSIS/Include
endif

CXXFLAGS += $(OPTFLAGS) -std=c++26
CXXFLAGS += -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics
CXXFLAGS += -funsigned-char -funsigned-bitfields -fshort-enums
CXXFLAGS += -ffunction-sections -fdata-sections -ftemplate-depth=2048
CXXFLAGS += -fstrict-aliasing -Wstrict-aliasing=1 -Wall -Wextra $(MCUFLAGS)

LDFLAGS += -Wl,--gc-sections
LDFLAGS += $(MCUFLAGS) -T$(LDSCRIPT)

BUILD_DIR = build/$(BOARD)
ifeq ($(TOOLCHAIN),clang)
CXX = clang++
CXXFLAGS := $(filter-out -funsigned-bitfields -Wstrict-aliasing=1,$(CXXFLAGS))
GCC_SYSROOT := $(shell arm-none-eabi-g++ -print-sysroot)
GCC_VERSION := $(shell arm-none-eabi-g++ -dumpfullversion)
GCC_MULTILIB := $(shell arm-none-eabi-g++ $(MCUFLAGS) -print-multi-directory)
CLANG_TARGET_FLAGS = --target=arm-none-eabi --gcc-toolchain=$(dir $(GCC_SYSROOT))
# Multilib headers and libraries are selected explicitly through GCC below.
CLANG_TARGET_FLAGS += -Wno-multilib-not-found
CXXFLAGS += $(CLANG_TARGET_FLAGS) -stdlib=libstdc++
CXXFLAGS += -isystem $(GCC_SYSROOT)/include/c++/$(GCC_VERSION)/arm-none-eabi/$(GCC_MULTILIB)
GCC_LIBDIR := $(dir $(shell arm-none-eabi-g++ $(MCUFLAGS) -print-libgcc-file-name))
NEWLIB_LIBDIR := $(dir $(shell arm-none-eabi-g++ $(MCUFLAGS) -print-file-name=libc_nano.a))
BUILD_DIR = build/$(BOARD)/clang
else
LDFLAGS += -Wl,--no-warn-rwx-segments -specs=nano.specs
endif

SOURCES = relay_01.cc $(SYSTEM).cc
OBJECTS = $(patsubst %.cc,$(BUILD_DIR)/%.o,$(SOURCES)) $(BUILD_DIR)/$(STARTUP).o
DEPS = $(patsubst %.cc,$(BUILD_DIR)/%.d,$(SOURCES))
ELF = $(BUILD_DIR)/relay_01.elf
BIN = $(BUILD_DIR)/relay_01.bin

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
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS) $(LOADLIBES)
endif

$(BIN): $(ELF)
	$(OBJCOPY) -O binary -S $< $@
	$(SIZE) $<

clean:
	$(RM) -r -- build

-include $(DEPS)
