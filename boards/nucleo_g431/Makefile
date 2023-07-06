CPU = cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

CPPFLAGS += -I../../../STM32CubeG4/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
CPPFLAGS += -I../../../STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc
CPPFLAGS += -DSTM32G431xx

LDSCRIPT = link.ld
LOADLIBES += startup.o system.o

targets += startup.o
targets += system.o
targets += test00.elf test00.S test00.bin
targets += test01.elf test01.bin

targets += fsk00.elf fsk00.bin
targets += fsk01.elf fsk01.bin

targets += sdr01.elf sdr01.bin

-include ../../Makefile.arm.include
