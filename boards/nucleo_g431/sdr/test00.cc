#define USE_MCU_STM_V1

#include "devices.h"

#include <chrono>

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;
    
    enum class State : uint8_t {Undefined, Init, StartConv, ReadResult};
    
    static inline constexpr External::Tick<systemTimer> mInitTicks{500ms};
    
    static inline void init() {
        devs::init();
        devs::led::set();
    }
    static inline void periodic() {
        ++r;
    }
    static inline void ratePeriodic() {
        
        const auto oldState = mState;
        
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
//            devs::dac1::set(0x2ff);
        break;
        case State::Init:
            if (devs::adc1::ready()) {
                mState = State::StartConv;
                devs::adc1::start();
            }
        break;
        case State::StartConv:
            if (!devs::adc1::busy()) {
                mState = State::ReadResult;
                ++ac;
            }
        break;
        case State::ReadResult:
            aValue = devs::adc1::value();
            mState = State::StartConv;
            devs::adc1::start();
        break;
        }

        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
            break;
            case State::Init:
            break;
            case State::StartConv:
            break;
            case State::ReadResult:
            break;
            }
        }
        
        ++a;
        a &= 0x0fff;
        devs::dac1::set(a);
        if (++c == 1000) {
            c = 0;
            IO::outl<trace>("systick: ", systemTimer::value, " r: ", r, " a: ", a);
            IO::outl<trace>("State: ", (uint8_t)mState, " adc: ", aValue, " ac: ", ac);
            r = 0;
        }
        devs::pinb4::toggle();
    }
private:
    static inline uint32_t c;
    static inline uint32_t r;
    static inline uint16_t a;
    
    static inline State mState{State::Undefined};
    static inline uint16_t aValue;
    static inline uint16_t ac;
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

