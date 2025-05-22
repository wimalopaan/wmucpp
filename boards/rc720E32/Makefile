# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-flash --reset --connect-under-reset write cc01.bin  0x8000000
# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace --trace=24000000     

CPU = cortex-m0plus
#FPU = -mfpu=softvfp
#FLOAT-ABI = -mfloat-abi=soft
OPTFLAGS = -g -Os

CPPFLAGS += -I../../include_stm32
CPPFLAGS += -I../../../STM32CubeG0/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../STM32CubeG0/Drivers/CMSIS/Device/ST/STM32G0xx/Include
CPPFLAGS += -I../../../STM32CubeG0/Drivers/STM32G0xx_HAL_Driver/Inc
CPPFLAGS += -DSTM32G0B1xx

LDSCRIPT = link_g0b1.ld
LOADLIBES += startup_g0b1.o system_g0.o

targets += startup_g0b1.o
targets += system_g0.o

#targets += rc720E32_01.elf rc720E32_01.bin
#targets += pult_01.elf pult_01.bin

targets += rc720E32_02.elf rc720E32_02.bin

-include ../../Makefile.arm.include
