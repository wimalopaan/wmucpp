#include "local.h"
#include "rcsensorled10.h"

using testPin0 = AVR::Pin<PortA, 0>; // LipoA1 = Chan0
using testPin1 = AVR::Pin<PortA, 1>; // LipoA2 = Chan1
using testPin2 = AVR::Pin<PortA, 2>; // LipoA3 = Chan2

using pins = AVR::PinSet<AVR::UsePgmTable, testPin0, testPin1, testPin2>;

int main() {
    pins::dir<AVR::Output>();
    pins::allOn();
    
    uintN_t<3> v;
    while(true) {
        pins::set(++v);
    }
}
