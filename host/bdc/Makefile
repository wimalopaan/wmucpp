#LOADLIBES += -liir

CPPFLAGS += -ICMSIS/DSP/Include 
CPPFLAGS += -ICMSIS/Include 
CPPFLAGS += -DHOST=ON

CXXFLAGS += -fpermissive -g

targets += test00 
targets += testDSP01 

test00: test00.cc mcu/etl/algorithm.h
testDSP01: testDSP01.cc

-include ../../Makefile.include
