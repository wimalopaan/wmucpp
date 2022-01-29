#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    using sm1 = devs::sm1;    
    using sm2 = devs::sm2;    
    using robo = devs::robo;    
    using sbus1 = devs::sbus1;    
    using sbus2 = devs::sbus2;    

    static inline void init() {
        devs::init();
        sm1::template init<AVR::BaudRate<38400>>();
        sm2::template init<AVR::BaudRate<38400>>();
        robo::template init<AVR::BaudRate<9600>>();    
        sbus1::init();
        sbus2::init();
    }
    static inline void periodic() {
        sm1::periodic();   
        sm2::periodic();   
        robo::periodic();   
        sbus1::periodic();   
        sbus2::periodic();   
    }
    static inline void ratePeriodic() {
        sbus1::ratePeriodic();   
        sbus2::ratePeriodic();   
    }
};

using devices = Devices<>;
using gfsm = GlobalFsm<devices>;

int main() {
    gfsm::init();    
    while(true) {
        gfsm::periodic(); 
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<devices::terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
//        terminalDevice::periodic();
//        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
