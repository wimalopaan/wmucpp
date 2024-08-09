#define USE_MCU_STM_V3
#define USE_CRSF_V2

#define NDEBUG

#include <cstdint>
#include <array>
#include <chrono>
#include <cassert>

#include "devicesPult.h"

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    // using led1 = devs::led1;
    // using led2 = devs::led2;

    // using tp1 = devs::tp1;
    // using tp2 = devs::tp2;

    // using i2c_2 = devs::i2c_2;

    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> initTicks{500ms};

    enum class State : uint8_t {Undefined, Init, Info, DevsInit, Run};

    static inline void init() {
        devs::init();
        // devs::led1::set();
    }
    static inline void periodic() {
        // tp1::set();
        trace::periodic();
        // i2c_2::periodic();
        // tp1::reset();
    }
    static inline void ratePeriodic() {
        // tp2::set();

        // i2c_2::ratePeriodic();

        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                // if (i2c_2::isIdle()) {
                //     mState = State::Info;
                // }
                // else {
                //     mStateTick.reset();
                // }
            });
            break;
        case State::Info:
            // for(uint8_t a{0}; a < 0x7f; ++a) {
            //     if (i2c_2::isPresent(I2C::Address{a})) {
            //         IO::outl<trace>("I2C2: ", a);
            //     }
            // }
            mState = State::DevsInit;
            break;
        case State::DevsInit:
            mStateTick.on(initTicks, []{
                // if (i2c_2::isIdle()) {
                //     mState = State::Run;
                // }
                // else {
                //     mStateTick.reset();
                // }
            });
            break;
        case State::Run:
            (++mDebugTick).on(debugTicks, []{
                // led1::toggle();
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                // i2c_2::scan([](Mcu::Stm::I2C::Address){});
                break;
            case State::Info:
                break;
            case State::DevsInit:
                break;
            case State::Run:
                // led2::set();
                break;
            }
        }
        // tp2::reset();
    }
    private:
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState{State::Undefined};
};

using devs = Devices<Pult00>;
using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    // NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    // __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
        // devs::tp1::set();
        // devs::tp1::reset();
    }
}

extern "C" {

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
