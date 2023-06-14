#include "devices.h"

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;
    
    static inline void init() {
        devs::init();
        devs::pinb8::set();
    }
    static inline void periodic() {
        ++r;
    }
    static inline void ratePeriodic() {
        ++a;
        a &= 0x0fff;
        devs::dac1::set(a);
        if (++c == 1000) {
            c = 0;
            IO::outl<trace>("systick: ", systemTimer::value, " r: ", r, " a: ", a);
            r = 0;
        }
        devs::pinb4::toggle();
    }
    
private:
    static inline uint32_t c;
    static inline uint32_t r;
    static inline uint16_t a;
};

//extern "C" void SysTick_Handler()  {                               
////    devs::systemTimer::isr();
//    devs::pinb3::set();
//    devs::pinb3::reset();
//}

int main() {
    using devs = Devices<void, Mcu::Stm::Stm32G431>;
    using gfsm = GFSM<devs>;
    gfsm::init();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
