MCU = atmega1284p
F_OSC = 20000000

all: main.elf main.size spiusart.elf spiusart.size
#all: main.elf main.hex main.size spiusart.elf spiusart.hex spiusart.size
all: simavr.elf simavr.size
#all: isrtest1.elf isrtest1.size test1.elf test1.size test2.elf test2.size test5.elf test5.size
#all: test10.elf test10.size test11.elf test11.size test12.elf test12.size
#all: spiusart.elf spiusart.hex spiusart.size

spiusart.elf spiusart.pgr: MCU = atmega328p
spiusart.elf spiusart.pgr: F_OSC = 16000000

simavr.elf: LDFLAGS += "-Wl,--section-start=.mmcu=0x910000" # special flags for simavr
simavr.elf: MCU = atmega328p
simavr.elf: F_OSC = 20000000
simavr.elf: simavr.o simavrconsole.o flash.o
	$(CC) $(LDFLAGS) -o $@ simavr.o simavrconsole.o flash.o

test10.elf: LDFLAGS += "-Wl,--section-start=.mmcu=0x910000" # special flags for simavr
test10.elf: MCU = atmega1284p
test10.elf: F_OSC = 20000000
test10.elf: test10.o simavrconsole.o
	$(CC) $(LDFLAGS) -o $@ test10.o simavrconsole.o


CCSRC = $(wildcard *.cc)
CCOBJ = $(patsubst %.cc, %.o, $(CCSRC))
CCDEP = $(patsubst %.cc, %.d, $(CCSRC))

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

LDFLAGS = $(MCUFLAGS) -Wl,-Map=main.map,--cref

AVRDUDE_PROGRAMMER = avrisp2
AVRDUDE_PORT = usb
#AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
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

%.size: %.elf
	avr-size -A $<

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
	$(RM) *.hex
	$(RM) *.elf
	$(RM) *.map
	$(RM) *.lss
	$(RM) *.eep


-include $(DEP)
