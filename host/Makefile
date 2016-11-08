
all: test01

ccsources = $(wildcard *.cc)
headers   = $(wildcard *.h)

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
