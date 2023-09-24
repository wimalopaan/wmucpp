
LOADLIBES += filter.o

targets += demod_fsk_01 demod_fsk_bp_01 sdr03 ppm01

ppm01: ppm01.cc mcu/algorithm.h mcu/dsp.h mcu/cppm.h mcu/fsk.h mcu/output.h

demod_fsk_01: demod_fsk_01.o filter.o

-include ../../Makefile.include
