#include "local.h"
#include "rcsensorled10.h"

using testPin0 = AVR::Pin<PortA, 0>; // LipoA1 = Chan0
using testPin1 = AVR::Pin<PortA, 1>; // LipoA2 = Chan1
using testPin2 = AVR::Pin<PortA, 2>; // LipoA3 = Chan2

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz * 256; 
}

int main() {
    testPin0::dir<AVR::Output>();
    
    hardPwm::init<Constants::pwmFrequency>();
    hbridge1::init();
    hbridge2::init();
    
    uint_ranged<uint8_t> v1 = 45;
    uint_ranged<uint8_t> v2 = 50;
    
    hbridge1::pwm(v1);
    hbridge2::pwm(v2);
    
    while(true) {
        testPin0::toggle();
    }
}
