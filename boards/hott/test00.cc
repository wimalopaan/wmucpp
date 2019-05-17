#define NDEBUG

#include <mcu/avr.h>

using PortB = AVR::Port<AVR::B>;
using dbg1 = AVR::Pin<PortB, 1>; 

int main() {
    dbg1::template dir<AVR::Output>();    
    
}
