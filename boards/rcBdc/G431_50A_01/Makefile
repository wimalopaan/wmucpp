# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-flash --reset --connect-under-reset write cc01.bin  0x8000000
# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace --trace=24000000     

# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace --trace=10m -v0 | tee -p p1 | gnuplot --persist
# grep '^#' p1 

CPU = cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

CPPFLAGS += -I../../../include_stm32
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc

CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/DSP/Include 
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/DSP/Source
CPPFLAGS += -ICMSIS/Include 

CPPFLAGS += -DSTM32G431xx

CPPFLAGS += -fpermissive

LDSCRIPT = link_g431.ld
LOADLIBES += startup_g431.o system_g4.o

targets += startup_g431.o
targets += system_g4.o

#targets += esc01.elf esc01.bin
#targets += esc05.elf esc05.bin
#targets += esc10.elf esc10.bin
targets += esc20.elf esc20.bin

-include ../../../Makefile.arm.include
