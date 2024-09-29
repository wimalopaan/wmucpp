# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-flash --reset --connect-under-reset write cc01.bin  0x8000000
# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace --trace=24000000     

CPU = cortex-m0plus
#FPU = -mfpu=softvfp
#FLOAT-ABI = -mfloat-abi=soft

CPPFLAGS += -I../../include_stm32
CPPFLAGS += -I../../../STM32CubeG0/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../STM32CubeG0/Drivers/CMSIS/Device/ST/STM32G0xx/Include
CPPFLAGS += -I../../../STM32CubeG0/Drivers/STM32G0xx_HAL_Driver/Inc
CPPFLAGS += -DSTM32G030xx

#CXXFLAGS += -g
OPTFLAGS = -O2

LDSCRIPT = link_g030.ld
LOADLIBES += startup_g030.o system_g0.o

targets += startup_g030.o
targets += system_g0.o

#targets += msw01.elf msw01.bin
#targets += msw02.elf msw02.bin
#targets += msw03.elf msw03.bin
#targets += msw04.elf msw04.bin
#targets += msw05.elf msw05.bin
#targets += msw06.elf msw06.bin
#targets += msw07.elf msw07.bin
#targets += msw08.elf msw08.bin
#targets += msw09.elf msw09.bin
#targets += msw10.elf msw10.bin
targets += msw11.elf msw11.bin
targets += msw12.elf msw12.bin

#targets += msw99.elf msw99.bin

-include ../../Makefile.arm.include
