# st-flash --reset --connect-under-reset write sdr04.bin  0x8000000
# st-trace --trace=24000000     

CPU = cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

CPPFLAGS += -I../include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc
CPPFLAGS += -DSTM32G431xx

LDSCRIPT = link.ld
LOADLIBES += startup.o system.o

targets += startup.o
targets += system.o

targets += rc01.elf rc01.S rc01.bin

-include ../../../Makefile.arm.include
