#define USE_MCU_STM_V1

#include "devices.h"

#include <chrono>

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;
    
//    enum class State : uint8_t {Undefined, Init, StartConv, ReadResult};
    
    static inline constexpr External::Tick<systemTimer> mInitTicks{500ms};
    
    static inline void init() {
        devs::init();
        devs::dac3::set(0x0fff / 2);
    }
    static inline void periodic() {
    }
    static inline void ratePeriodic() {
        ++a;
        a &= 0x0fff;
        devs::dac1::set(a);
        
        if (++c == 1000) {
            c = 0;
            IO::outl<trace>("systick: ", systemTimer::value, " a: ", a);
        }
    }
private:
    static inline uint32_t c;
//    static inline uint32_t r;
    static inline uint16_t a;
    
//    static inline State mState{State::Undefined};
//    static inline uint16_t aValue;
//    static inline uint16_t ac;
};

//extern "C" void SysTick_Handler()  {                               
////    devs::systemTimer::isr();
//    devs::pinb3::set();
//    devs::pinb3::reset();
//}

int main() {
    using devs = Devices<Test01, Mcu::Stm::Stm32G431>;
    using gfsm = GFSM<devs>;
    gfsm::init();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

