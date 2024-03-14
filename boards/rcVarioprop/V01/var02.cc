#define USE_MCU_STM_V3
#define NDEBUG

#include "devices_02.h"

#include <chrono>
#include <cassert>

using namespace std::literals::chrono_literals;

struct Config {
    Config() = delete;
};

struct Data {
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    // using led = devs::led;
    // using btPwr = devs::btPwr;
    // using hfPwr = devs::hfPwr;

           // using adc1 = devs::adc1;
           // using adc2 = devs::adc2;

    // using i2c1 = devs::i2c1;
    // using si = devs::si;

    using cppm = devs::cppm;

    static inline void init() {
        devs::init();
        // led::set();
        // btPwr::reset();
    }
    static inline void periodic() {
        // trace::periodic();
        // si::periodic();
        // i2c1::periodic();
    }

    enum class State : uint8_t {Undefined, Run};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> changeTicks{2000ms};

    static inline void ratePeriodic() {
        const auto oldState = mState;

        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            // (++mChangeTick).on(changeTicks, []{
            //     cppm::set(1, mToggle ? 100 : 900);
            //     mToggle = !mToggle;
            // });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Run:
                break;
            }
        }
    }

    static inline uint32_t div{0};
private:
    static inline bool mToggle{false};
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mChangeTick;
};

using devs = Devices<Var01, Config, Mcu::Stm::Stm32G431>;

int main() {
    using gfsm = GFSM<devs>;
    gfsm::init();

    // NVIC_EnableIRQ(ADC1_2_IRQn);
    // NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    // NVIC_EnableIRQ(TIM3_IRQn);
    // NVIC_EnableIRQ(EXTI15_10_IRQn);
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
    }
}

extern "C" {

// void DMA1_Channel1_IRQHandler() {
//     DMA1->IFCR = DMA_IFCR_CTCIF1;
// }
// void DMA1_Channel3_IRQHandler() {
//     DMA1->IFCR = DMA_IFCR_CTCIF3;
//     // devs::tp0::set();
//     // devs::tp0::reset();
// }

}
