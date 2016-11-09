targets = test01

all: $(targets)

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

CXX = clang++

STDFLAGS += -std=c++1z
CXXFLAGS += -I../include
CXXFLAGS += -g -Wall -Wextra -Wconversion 

%.d: %.cc
	$(CXX) -MM -MG $(CXXFLAGS) $< | sed 's/$*\.o/& $@ $<.extract/g' > $@

%.o: %.cc
	$(COMPILE.cc) $(STDFLAGS) $(OUTPUT_OPTION) $(shell pwd)/$<

#%: %.cc
#	$(COMPILE.cc) $(STDFLAGS) $(OUTPUT_OPTION) $(shell pwd)/$<

%: %.cc
	$(CXX) $(CXXFLAGS) $(STDFLAGS) $(OUTPUT_OPTION) $(shell pwd)/$<

clean:
	$(RM) $(OBJ)
	$(RM) $(DEP)
	$(RM) $(targets)
	$(RM) *.hex
	$(RM) *.elf
	$(RM) *.map
	$(RM) *.lss
	$(RM) *.eep
