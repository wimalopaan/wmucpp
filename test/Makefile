MCU = atmega1284p
F_OSC = 20000000

all: test.s


CCSRC = $(wildcard *.cc)
CCOBJ = $(patsubst %.cc, %.o, $(CCSRC))
CCDEP = $(patsubst %.cc, %.d, $(CCSRC))
CCASM = $(patsubst %.cc, %.s, $(CCSRC))

CSRC = $(wildcard *.c)
COBJ = $(patsubst %.c, %.o, $(CSRC))
CDEP = $(patsubst %.c, %.d, $(CSRC))

SRC += $(CCSRC) 
SRC += $(CSRC) 

OBJ += $(CCOBJ)
OBJ += $(COBJ)

DEP += $(CCDEP)
DEP += $(CDEP)

MCUFLAGS = -mmcu=$(MCU)

CXXFLAGS += -DNDEBUG
CXXFLAGS += -Os -std=c++1z
#CXXFLAGS += -nostdlib -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics
CXXFLAGS += -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics #-Wgnu-string-literal-operator-template
CXXFLAGS += -funsigned-char -funsigned-bitfields -fshort-enums
CXXFLAGS += -Wall -Wextra
CXXFLAGS += -DF_OSC=$(F_OSC) -DF_CPU=$(F_OSC)
CXXFLAGS += -I. -I../include
#CXXFLAGS += -I/usr/include/simavr
CXXFLAGS += $(MCUFLAGS)

CFLAGS += -Os -std=gnu11
#CFLAGS += -Os -std=c11
CFLAGS += -funsigned-char -funsigned-bitfields -fshort-enums
CFLAGS += -DF_OSC=$(F_OSC) -DF_CPU=$(F_OSC)
CFLAGS += -I. -I../include
CFLAGS += -I/usr/include/simavr
CFLAGS += $(MCUFLAGS)

#ASMFLAGS += -fverbose-asm
ASMFLAGS +=  -Wa,-adhln

LDFLAGS = $(MCUFLAGS) -Wl,-Map=main.map,--cref

CC = avr-gcc
CXX = avr-g++
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude

%.s: %.cc
	$(CXX) $(CXXFLAGS) $(ASMFLAGS) -S -o $@ $<

%.size: %.elf
	avr-size -A $<

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
	$(RM) $(CCASM)
	$(RM) $(OBJ)
	$(RM) $(DEP)
	$(RM) *.hex
	$(RM) *.elf
	$(RM) *.map
	$(RM) *.lss
	$(RM) *.eep


-include $(DEP)
