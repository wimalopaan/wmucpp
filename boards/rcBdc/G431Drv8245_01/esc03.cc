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
    // using tp = devs::tp;

    using pwm = devs::pwm;
    using nsleep = devs::nsleep;
    using nsleepPulseWaiter = devs::nsleepPulseWaiter;
    using in1 = devs::in1;
    using in2 = devs::in2;

    using adc = devs::adc;
    using dac = devs::dac;

    static inline void init() {
        devs::init();
        led::set();
    }

    enum class State : uint8_t {Undefined, Init, Run, Reset};

    static inline void periodic() { // 300ns
        // devs::tp::set();
        // devs::tp::reset();
        trace::periodic();

        switch(mState) {
        case State::Undefined:
            break;
        case State::Init:
            break;
        case State::Run:
            if (!adc::busy()) { // ändern: nach ganzer Sequenz
                devs::tp::set();
                const uint16_t v = adc::value();
                dac::set2(v);
                devs::tp::reset();
            }
            break;
        case State::Reset:
            break;
        }
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};

    static inline void endReset() {
        adc::start();
        mState = State::Init;
    }

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
            mStateTick.on(initTicks, []{
                // IO::outl<trace>("a: ", aaa);
            });
            break;
        case State::Reset:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                adc::start();
                //nsleep::set();
                // dac::set2(2048);
                break;
            case State::Run:
                // in1::afunction(2);
                in2::afunction(2);
                pwm::duty(1400);
                led::reset();
                break;
            case State::Reset:
                nsleep::reset();
                nsleepPulseWaiter::start();
                break;
            }
        }
    }

private:
    static inline volatile State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
};

using devs = Devices<ESC03, Config, Mcu::Stm::Stm32G431>;
using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    // NVIC_EnableIRQ(ADC1_2_IRQn);
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
        devs::tp::set();
        devs::tp::reset();
    }
}

extern "C" {

void ADC1_2_IRQHandler() {
    // devs::tp::set();
    ADC1->ISR = ADC_ISR_EOC;
    // const uint16_t v = devs::adc::value();
    // devs::dac::set2(v);
    // aaa = v;
    // devs::tp::reset();
}

void TIM6_DAC_IRQHandler() {
    TIM6->SR &= ~TIM_SR_UIF;
    devs::nsleep::set();
    devs::nsleepPulseWaiter::stop();
    gfsm::endReset();
}

void TIM3_IRQHandler() {
    devs::tp::set();
    devs::adc::start();
    TIM3->SR &= ~TIM_SR_CC3IF;
    TIM3->SR &= ~TIM_SR_CC1IF;
    devs::tp::reset();
}

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
