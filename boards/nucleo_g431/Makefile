CPU = cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

CPPFLAGS += -I../../../STM32CubeG4/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
CPPFLAGS += -DSTM32G431xx

LDSCRIPT = link.ld
LOADLIBES += startup.o system.o

targets += startup.o
targets += system.o
targets += test00.elf test00.S

-include ../../Makefile.arm.include
