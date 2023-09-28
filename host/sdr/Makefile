
LOADLIBES += filter.o

targets += demod_fsk_01 demod_fsk_bp_01 sdr03 ppm01

ppm01: ppm01.cc mcu/include/algorithm.h mcu/include/dsp.h mcu/include/cppm.h mcu/include/fsk.h mcu/include/output.h

demod_fsk_01: demod_fsk_01.o filter.o

-include ../../Makefile.include
