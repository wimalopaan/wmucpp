# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-flash --reset --connect-under-reset write cc01.bin  0x8000000
# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace --trace=24000000     

CPU = cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

# CXXFLAGS += -g
OPTFLAGS = -Os

CPPFLAGS += -I../../../include_stm32
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc

CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/DSP/Include 
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/DSP/Source
CPPFLAGS += -ICMSIS/Include 

CPPFLAGS += -DSTM32G431xx

CXXFLAGS += -fpermissive

LDSCRIPT = link_g431.ld
LOADLIBES += startup_g431.o system_g4.o

targets += startup_g431.o
targets += system_g4.o

targets += esc30.elf esc30.bin

-include ../../../Makefile.arm.include
