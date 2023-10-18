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

targets += esc01.elf esc01.S esc01.bin
targets += esc_hall_01.elf esc_hall_01.bin
targets += esc_hall_02.elf esc_hall_02.bin

-include ../../../Makefile.arm.include
