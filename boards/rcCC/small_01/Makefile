# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-flash --reset --connect-under-reset write sdr04.bin  0x8000000
# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace --trace=24000000     
# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace  

CPU = cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

CPPFLAGS += -I../../../include_stm32
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc
CPPFLAGS += -DSTM32G431xx
#CPPFLAGS += -g

LDSCRIPT = link_g431.ld
LOADLIBES += startup_g431.o system_g4.o

targets += startup_g431.o
targets += system_g4.o

targets += cc01.elf cc01.bin
targets += cc02.elf cc02.bin
targets += cc03.elf cc03.bin

-include ../../../Makefile.arm.include
