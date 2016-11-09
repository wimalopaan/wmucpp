MCU = atmega328p
F_OSC = 20000000

all: bm01a.s bm01a.elf bm01a.size
all: bm01b.s bm01b.elf bm01b.size
all: bm02a.s bm02a.elf bm02a.size
all: bm02b.s bm02b.elf bm02b.size
all: bm03a.s bm03a.elf bm03a.size
all: bm03b.s bm03b.elf bm03b.size
#all: bm04a.s bm04a.elf bm04a.size
#all: bm04b.s bm04b.elf bm04b.size
#all: bm05a.s bm05a.elf bm05a.size
#all: bm05b.s bm05b.elf bm05b.size
#all: bm06a.s bm06a.elf bm06a.size
#all: bm06b.s bm06b.elf bm06b.size

bm02a.elf: bm02a.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ bm02a.o simavrconsole.o

bm02b.elf: bm02b.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ bm02b.o simavrconsole.o

CCSRC = $(wildcard *.cc)
CCOBJ = $(patsubst %.cc, %.o, $(CCSRC))
CCDEP = $(patsubst %.cc, %.d, $(CCSRC))
CCASM = $(patsubst %.cc, %.s, $(CCSRC))

CSRC = $(wildcard *.c)
COBJ = $(patsubst %.c, %.o, $(CSRC))
CDEP = $(patsubst %.c, %.d, $(CSRC))
CASM = $(patsubst %.c, %.s, $(CSRC))

SRC += $(CCSRC) 
SRC += $(CSRC) 

OBJ += $(CCOBJ)
OBJ += $(COBJ)

ASM += $(CCASM)
ASM += $(CASM)

DEP += $(CCDEP)
DEP += $(CDEP)

MCUFLAGS = -mmcu=$(MCU)

SIMAVR = ~/Projekte/simavr/simavr/run_avr

CXXFLAGS += -DNDEBUG
CXXFLAGS += -Os -std=c++1z
#CXXFLAGS += -nostdlib -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics
CXXFLAGS += -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics #-Wgnu-string-literal-operator-template
CXXFLAGS += -funsigned-char -funsigned-bitfields -fshort-enums
CXXFLAGS += -Wall -Wextra
CXXFLAGS += -DF_OSC=$(F_OSC) -DF_CPU=$(F_OSC)
CXXFLAGS += -I. -I../../include -I../../include/std
CXXFLAGS += -I/usr/include/simavr
CXXFLAGS += -I/usr/avr/include
CXXFLAGS += $(MCUFLAGS)

CFLAGS += -DNDEBUG
#CFLAGS += -Os -std=gnu11
CFLAGS += -Os -std=c11
CFLAGS += -funsigned-char -funsigned-bitfields -fshort-enums
CFLAGS += -Wall -Wextra
CFLAGS += -DF_OSC=$(F_OSC) -DF_CPU=$(F_OSC)
CFLAGS += -I. -I../../include
CFLAGS += -I/usr/include/simavr
CFLAGS += -I/usr/avr/include
CFLAGS += $(MCUFLAGS)

#ASMFLAGS += -fverbose-asm
ASMFLAGS +=  -Wa,-adhln

LDFLAGS = $(MCUFLAGS) -Wl,-Map=main.map,--cref
LDFLAGS += "-Wl,--section-start=.mmcu=0x910000"

AVRDUDE_PROGRAMMER = avrisp2
AVRDUDE_PORT = usb
AVRDUDE_NO_VERIFY = -V
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)
CC = avr-gcc
CXX = avr-g++
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude

%.s: %.cc
	$(CXX) $(CXXFLAGS) $(ASMFLAGS) -S -o $@ $<

%.s: %.c
	$(CC) $(CFLAGS) $(ASMFLAGS) -S -o $@ $<

%.size: %.elf
	avr-size $< > $@

# Program the device.  
%.pgr: %.hex %.eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$< $(AVRDUDE_WRITE_EEPROM)

# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

%.eep: %.elf
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O ihex $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	$(NM) -n $< > $@

%.elf: %.cc
	$(CXX) $(CXXFLAGS) $< --output $@ $(LDFLAGS)

%.elf: %.c
	$(CC) $(CFLAGS) $< --output $@ $(LDFLAGS)

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

%.d: %.cc
	$(CXX) -MM -MG $(CXXFLAGS) $< | sed 's/$*\.o/& $@ $*.elf $<.extract/g' > $@

clean:
	$(RM) $(OBJ)
	$(RM) $(DEP)
	$(RM) $(ASM)
	$(RM) *.hex
	$(RM) *.elf
	$(RM) *.map
	$(RM) *.lss
	$(RM) *.eep
	$(RM) *.size


-include $(DEP)
