#define USE_MCU_STM_V3
#define NDEBUG

#include "devices.h"

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

    using led = devs::led;
    using btPwr = devs::btPwr;
    using hfPwr = devs::hfPwr;

    using adc1 = devs::adc1;
    // using adc2 = devs::adc2;

    using i2c1 = devs::i2c1;
    using si = devs::si;

    using cppm = devs::cppm;

    static inline void init() {
        devs::init();
        led::set();
        btPwr::reset();
    }
    static inline void periodic() {
        trace::periodic();
        si::periodic();
        i2c1::periodic();
    }

    enum class State : uint8_t {Undefined, Init, Run, Reset, Set, Set2};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

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
                si::setOutput(0);
                mState = State::Set;
            });
            break;
        case State::Set:
            // Channel{Band::_40MHz, 52, 40'685'000}, // 50
            if (si::setFrequency(40'875'000_Hz, &div)) {
                si::setOutput(2);
                mState = State::Set2;
            }
            break;
        case State::Set2:
            if (si::setFrequency(40'879'000_Hz)) { // 4KHz
                mState = State::Run;
            }
            break;
        case State::Run:
            (++mDebugTick).on(debugTicks, []{
                IO::outl<trace>("a0: ", adc1::mData[0], "a1: ", adc1::mData[1], "a2: ", adc1::mData[2], "a3: ", adc1::mData[3]);

            });
            {
                uint16_t v = 999.0f * adc1::mData[0] / 4095.0f;
                cppm::set(0, v);
            }
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
                adc1::start();
                hfPwr::set();
                // led::reset();
                break;
            case State::Reset:
                break;
            }
        }
    }

    static inline uint32_t div{0};
private:
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
};

using devs = Devices<Var01, Config, Mcu::Stm::Stm32G431>;

int main() {
    using gfsm = GFSM<devs>;
    gfsm::init();

    NVIC_EnableIRQ(ADC1_2_IRQn);
    __enable_irq();

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

static volatile uint8_t ii = 0;

void ADC1_2_IRQHandler() {
    if (ADC1->ISR & ADC_ISR_EOS) {
        ADC1->ISR = ADC_ISR_EOS;
    }
    if (ADC1->ISR & ADC_ISR_EOC) {
        ADC1->ISR = ADC_ISR_EOC;
    }
    if (ADC2->ISR & ADC_ISR_EOS) {
        ADC2->ISR = ADC_ISR_EOS;
    }
    if (ADC2->ISR & ADC_ISR_EOC) {
        ADC2->ISR = ADC_ISR_EOC;
    }
}

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
