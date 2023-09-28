# st-flash --reset --connect-under-reset write sdr04.bin  0x8000000
# st-trace

CPU = cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

CPPFLAGS += -I../include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc
CPPFLAGS += -DSTM32G431xx

#CXXFLAGS += -ffreestanding

LDSCRIPT = link.ld
LOADLIBES += startup.o system.o

targets += startup.o
targets += system.o
#targets += test00.elf test00.S test00.bin
#targets += test01.elf test01.bin

#targets += fsk00.elf fsk00.bin
#targets += fsk01.elf fsk01.bin

#targets += sdr01.elf sdr01.bin sdr01.S
#targets += sdr02.elf sdr02.bin sdr02.S
targets += sdr03.elf sdr03.bin sdr03.S
targets += sdr04.elf sdr04.bin sdr04.S

targets += cppm01.elf cppm01.bin cppm01.S
targets += cppm02.elf cppm02.bin cppm02.S

targets += test99.elf test99.S
targets += test100.elf test100.S
targets += test101.elf test101.S

-include ../../../Makefile.arm.include
