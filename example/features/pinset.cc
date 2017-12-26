#define NDEBUG

#include "mcu/ports.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using p0 = AVR::Pin<PortB, 0>;
using p1 = AVR::Pin<PortB, 1>;

using pinset = AVR::PinSet<p1, p0>;

bitsN_t<2> x;

int main() {
    pinset::dir<AVR::Output>();

    pinset::set(x);    
//        pinset::set<3>();    
}
