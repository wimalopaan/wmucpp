#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
//#include <mcu/internals/sigrow.h>
#include <mcu/internals/twi.h>
#include <mcu/internals/eeprom.h>

#include <external/solutions/blinker.h>

namespace { 
    using namespace std::literals::chrono;
    using namespace External::Units::literals;
    inline static constexpr auto fRtc = 1000_Hz;
}

template<auto HWRev = 0, typename MCU = DefaultMcuType>
struct Devices;

// HWRev 5
template<typename MCU>
struct Devices<4, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
//    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;

    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 3>, AVR::Output>; 
    using blinkLed1 = External::Blinker2<led1, systemTimer, 200_ms, 2000_ms>;
    using b_t = blinkLed1::count_type;
    
    static inline void init() {
        ccp::unlock([]{
            clock::prescale<1>(); 
        });
        systemTimer::init(); 
    }
};

using devs = Devices<4>;

int main() {
    devs::init();
    devs::blinkLed1::init();
    devs::blinkLed1::blink(devs::b_t{4});
    while(true) {
        devs::systemTimer::periodic([&]{
            devs::blinkLed1::ratePeriodic();
        });
    }
    
}
