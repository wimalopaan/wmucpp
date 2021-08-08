#pragma once

struct NoBoard1614 {
    // tca:wo0 = pb0
    // lut0    = pa4
    // tca:wo1 = pb1
    // lut1    = pa7
    
    // an0      = pa1
    // an1      = pa2
    // bn0      = pa5
    // bn1      = pa6
    
    // ppm      = pb2
    
    // ibus/sbus= pb3
    // tx       = pb2
};
struct NoBoard412 {
    // tca:wo0 = pa3
    // tca:wo1 = pa1
    
    // an0      = pa2
    // bn0      = pa6
    
    // ibus/sbus= pa7 (ain7/ppm)
    // tx       = (pa6)
};

template<typename Board, typename MCU = DefaultMcuType> 
struct Devices;

#if defined(__AVR_ATtiny1614__)
# include "devices1614.h"
#endif

#if defined(__AVR_ATtiny412__)
# include "devices412.h"
#endif
