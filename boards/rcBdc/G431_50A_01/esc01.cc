#define USE_MCU_STM_V2
#define NDEBUG

#include "devices.h"

#include <chrono>
#include <cassert>

using namespace std::literals::chrono_literals;

struct Config {
    Config() = delete;
};

struct Data {
//    static inline std::array<uint16_t, 64> mChannels{}; // sbus [172, 1812], center = 992
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    using led = devs::led;

    using pwm = devs::pwm;
    using pwm3 = devs::pwm3;

    using adc = devs::adc;
    using dac = devs::dac;

    static inline void init() {
        devs::init();
        led::set();
    }
    static inline void periodic() {
        trace::periodic();

        // if (!adc::busy()) {
        // }
    }

    enum class State : uint8_t {Undefined, Init, Run, Reset};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};

    static inline void ratePeriodic() {
        const auto oldState = mState;

        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Reset;
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            break;
        case State::Reset:
            mState = State::Init;
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                break;
            case State::Run:
                pwm::duty(1500);
                pwm3::set();
                // led::reset();
                break;
            case State::Reset:
                break;
            }
        }
    }

private:
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
};

using devs = Devices<ESC01, Config, Mcu::Stm::Stm32G431>;

int main() {
    using gfsm = GFSM<devs>;
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
        devs::tp2::set();
        devs::tp2::reset();
    }
}

extern "C" {

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
