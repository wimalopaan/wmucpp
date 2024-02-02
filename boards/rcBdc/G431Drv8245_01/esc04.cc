#define USE_MCU_STM_V3
#define NDEBUG

namespace Mcu::Stm {
inline namespace V3{}
}


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

struct Estimator {
    static inline Dsp::HystereseThreshold<void> th{0.0001};

           // static inline auto expMean = []{
           //     Dsp::ExpMean<void> filter{0.0001};
           //     return filter;
           // }();

    static inline auto iirFilter = []{
        Dsp::Butterworth::LowPass<6> filter;
        filter.setup(20'000, 200);
        // Iir::Butterworth::LowPass<6> filter;
        // // filter.setup(Config::fs, 200, 5);
        // filter.setup(Config::fs, 200);
        return filter;
    }();

    static inline void process(const float v) {
        const auto x = iirFilter.process(v);
        v1 = x;
        const bool t = th.process(x);

        ++pCounter;
        if (!last && t) { // rising
            period = pCounter;
            pCounter = 0;
        }
        last = t;
    }
    static inline bool last;
    static inline uint32_t pCounter;
    static inline uint32_t period;
    static inline float v1;
};

using estimator = Estimator;


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
    using adcDmaChannel = devs::adcDmaChannel;

    using dac = devs::dac;

    using servo = devs::servo;
    using servo_pa = devs::servo_pa;
    using sensor = devs::ibus_sensor;

    static inline void init() {
        devs::init();
        led::set();
    }

    enum class State : uint8_t {Undefined, Init, Run, Reset};

    static inline Dsp::ExpMean<void> rpmEst{0.3};

    static inline void periodic() { // 300ns
        // devs::tp::set();
        // devs::tp::reset();
        trace::periodic();

        servo::periodic();
        sensor::periodic();

        switch(mState) {
        case State::Undefined:
            break;
        case State::Init:
            break;
        case State::Run:
            if (adc::gotSequence()) {
                const float c = adc::mData[0];
                estimator::process(c);
                dac::set2(estimator::v1);
                // dac::set2(adc::mData[0]);

            }
            break;
        case State::Reset:
            break;
        }
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemTicks{50ms};

    static inline void endReset() {
        adc::start();
        mState = State::Init;
    }

    using ch_t = servo_pa::channel_type;

    static inline void ratePeriodic() {
        const auto oldState = mState;

        servo_pa::ratePeriodic();
        sensor::ratePeriodic();


        ++mStateTick;
        ++mTelemTick;
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
            mTelemTick.on(telemTicks, []{
                if (estimator::period > 0) {
                    if (estimator::period > 20000) {
                        RpmProvider::mValue = 0;
                    }
                    else {
                        RpmProvider::mValue = rpmEst.process(60.0f * (20'000.0f / estimator::period) / 2.0f);
                    }
                }
                float d = abs(servo_pa::value(ch_t{0}).toInt() - 1500);
                d *= 1640;
                d /= 512;
                pwm::duty(d);

                VoltageProvider::mValue = adc::mData[1];

                CurrentProvider::mValue = estimator::th.mMean.value();
            });

            mStateTick.on(initTicks, []{
                IO::outl<trace>("a0: ", adc::mData[0], " a1: ", adc::mData[1], " p: ", estimator::period);
                IO::outl<trace>("c0: ", servo_pa::value(ch_t{0}).toInt());
                IO::outl<trace>("r: ", sensor::ProtocollAdapter::requests(), " b: ", sensor::StatisticProvider::mBytes);
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
                break;
            case State::Run:
                // in1::afunction(2);
                in2::afunction(2);
                pwm::duty(400);
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
    static inline External::Tick<systemTimer> mTelemTick;
};

using devs = Devices<ESC04, Config, Mcu::Stm::Stm32G431>;
using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    // NVIC_EnableIRQ(TIM3_IRQn);
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
    // devs::tp::reset();
}

void TIM6_DAC_IRQHandler() {
    TIM6->SR &= ~TIM_SR_UIF;
    devs::nsleep::set();
    devs::nsleepPulseWaiter::stop();
    gfsm::endReset();
}

void TIM3_IRQHandler() {
    // devs::tp::set();
    // devs::adc::start();
    TIM3->SR &= ~TIM_SR_CC3IF;
    // devs::tp::reset();
}

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
