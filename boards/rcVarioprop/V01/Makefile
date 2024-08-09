# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-flash --reset --connect-under-reset write cc01.bin  0x8000000
# LD_PRELOAD=/usr/local/lib/libstlink.so /usr/local/bin/st-trace --trace=24000000     

CPU = cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

CPPFLAGS += -I../../../include_stm32
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Core/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
CPPFLAGS += -I../../../../STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc
CPPFLAGS += -DSTM32G431xx

#OPTFLAGS += -g

LDSCRIPT = link_g431.ld
LOADLIBES += startup_g431.o system_g4.o

targets += startup_g431.o
targets += system_g4.o

#targets += var01.elf var01.bin
#targets += var02.elf var02.bin
#targets += var03.elf var03.bin
#targets += var04.elf var04.bin
#targets += var05.elf var05.bin
#targets += var06.elf var06.bin
#targets += var07.elf var07.bin
#targets += var08.elf var08.bin
#targets += var09.elf var09.bin
#targets += var10.elf var10.bin
#targets += var11.elf var11.bin
targets += var12.elf var12.bin

-include ../../../Makefile.arm.include
