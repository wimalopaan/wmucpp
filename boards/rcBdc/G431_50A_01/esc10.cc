#define USE_MCU_STM_V3
#define NDEBUG

namespace Mcu::Stm {
inline namespace V3{}
}

#include "devices.h"

#include <chrono>
#include <cassert>

#include "cmsis.h"

using namespace std::literals::chrono_literals;

namespace detail {
    template<typename T> concept ConceptPwm = requires(T) { T::pwm(0u); };
    template<typename T> struct hasPwm : std::integral_constant<bool, false> {};
    template<ConceptPwm T> struct hasPwm<T> : std::integral_constant<bool, true> {};
    template<typename T> concept ConceptCutoff= requires(T) { T::cutoff(0u); };
    template<typename T> struct hasCutoff: std::integral_constant<bool, false> {};
    template<ConceptCutoff T> struct hasCutoff<T> : std::integral_constant<bool, true> {};
}

template<typename... Listener>
struct Config {
    using listeners = Meta::List<Listener...>;
    using pwmListeners = Meta::filter<detail::hasPwm, listeners>;
    using cutoffListeners = Meta::filter<detail::hasCutoff, listeners>;
    // using pwmListeners::_;
    // using cutoffListeners::_;

    static inline uint16_t fPwm = 32'000;
    static inline uint16_t fCutoff = 1'000;
    // static inline uint16_t fCutoff = 200;
    static inline uint16_t maxRpm = 10'000;

    static inline void servo(const uint16_t s) {
    }

    static inline void init() {
        pwm(fPwm);
        cutoff(fCutoff);
    }

    static inline void pwm(const uint16_t f) {
        fPwm = f;
        [&]<typename... L>(Meta::List<L...>){
            (L::pwm(f), ...);
        }(pwmListeners{});
    }
    static inline void cutoff(const uint16_t f) {
        fCutoff = f;
        [&]<typename... L>(Meta::List<L...>){
            (L::cutoff(f), ...);
        }(cutoffListeners{});
    }
};

struct Estimator {
    // static inline Dsp::HystereseThreshold<void> th{0.001};
    static inline Dsp::HystereseThreshold<void> th{0.005};
    static inline Dsp::Butterworth::LowPass<6> iirFilter;

    static inline void cutoff(const uint16_t f) {
        iirFilter.fc(f);
    }

    static inline void pwm(const uint16_t f) {
        iirFilter.fs(f);
        // th anpassen
    }

    static inline void process(const float v) {
        const auto x = iirFilter.process(v);
        v1 = x;
        const bool t = th.process(x);

        // todo: mehrere Perioden, da Perioden teilw. unterschiedlich lang
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


template<uint16_t Size, typename Source>
struct FFTEstimator {
    static inline void init() {
        arm_status status = arm_rfft_fast_init_f32(&fftInstance, Size);
    }
    static inline void update() {
        __disable_irq();
        std::copy(std::begin(Source::data), std::end(Source::data), samples);
        __enable_irq();

        arm_rfft_fast_f32(&fftInstance, &samples[0], &fft[0], 0); // fft[0] (real) : dc-offset
        arm_cmplx_mag_f32(&fft[0], &magnitude[0], Size / 2); // interleaved (real, complex) input, normal output
        float maxValue{};
        arm_max_f32(&magnitude[0], Size / 2, &maxValue, &maxIndex);
    }
// private:
    static inline float maxValue{};
    static inline uint32_t maxIndex{};
    static inline arm_rfft_fast_instance_f32 fftInstance{};
    static inline std::array<float, Size> samples{};
    static inline std::array<float, Size> fft{};
    static inline std::array<float, Size / 2> magnitude{};
};

// using estimator = Estimator;

using devs = Devices<ESC10, void, Mcu::Stm::Stm32G431>;

template<typename Adc>
struct SubSampler {
    using adc = Adc;
    static inline void cutoff(const uint16_t f) {
        iirFilter.fc(f);
    }

    static inline void pwm(const uint16_t f) {
        iirFilter.fs(f);
    }
    static inline void isr() {
        const auto x = iirFilter.process(adc::mData[0]);
        if (++sampleCounter == factor) {
            sampleCounter = 0;
            data.push_back(x);
        }
    }
// private:
    static inline etl::FiFo<volatile float, 2048> data;
    static inline volatile uint16_t sampleCounter{0};
    static inline volatile uint16_t factor{10};
    static inline Dsp::Butterworth::LowPass<6, volatile float> iirFilter;
};

using subSampler = SubSampler<devs::adc>;
using estimator = FFTEstimator<2048, subSampler>;
using config = Config<estimator, subSampler, devs::pwm>;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    using led = devs::led;
    using tp1 = devs::tp1;
    using tp2 = devs::tp2;

    using pwm = devs::pwm;
    using pwm3 = devs::pwm3;

    using adc = devs::adc;
    using adcDmaChannel = devs::adcDmaChannel;
    using pga = devs::pga;

    using dac = devs::dac;

    using servo = devs::servo;
    using servo_pa = devs::servo_pa;
    using sensor = devs::ibus_sensor;

    static inline void init() {
        devs::init();
        led::set();

        config::init();
    }

    enum class State : uint8_t {Undefined, Init, Run};

    static inline Dsp::ExpMean<void> rpmEst{0.3};

    static inline void periodic() {
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
            adc::whenSequenceComplete([]{
                tp2::set();
                const float c = adc::mData[0];
                // estimator::process(c);
                // dac::set(std::min(estimator::v1, 4095.0f));
                // dac::set(adc::mData[0]);
                // dac::set(std::min(estimator::th.mMean.value(), 4095.0f));

                // if (estimator::last) {
                //     tp1::set();
                // }
                // else {
                //     tp1::reset();
                // }
                tp2::reset();
            });
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
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            mTelemTick.on(telemTicks, []{
                estimator::update();


                // if (estimator::period > 0) {
                //     if (estimator::period > 20000) {
                //         RpmProvider::mValue = 0;
                //     }
                //     else {
                //         RpmProvider::mValue = rpmEst.process(60.0f * (20'000.0f / estimator::period) / 7.0f); // 7-Pole
                //     }
                // }
                const auto nv = servo_pa::normalized(ch_t{0});
                // float d = abs(servo_pa::value(ch_t{0}).toInt() - 1500);
                // d *= 1640;
                // d /= 512;
                // dd = d;
                // pwm::duty(dd);
                // pwm::normalized(nv);

                VoltageProvider::mValue = adc::mData[1];

                // CurrentProvider::mValue = estimator::th.mMean.value() / 10.0f;


                // [min;max] range

                // const float r = estimator::th.mMax.value() - estimator::th.mMin.value();

                // nur basierend auf mMax

                // if (r > (4095.0f * 0.9)) {
                //     if (gain > 0) gain -= 1;
                // }
                // if (r < (4095.0f * 0.1)) {
                //     if (gain < 5) gain += 1;
                // }
                // if (estimator::th.mMean.value() > (4095.0f * 0.9)) {
                //     if (gain > 0) gain -= 1;
                // }
                // else if (estimator::th.mMean.value() < (4095.0f * 0.1)) {
                //     if (gain < 5) gain += 1;
                // }
                pga::gain(gain);

            });

            mStateTick.on(initTicks, []{
                // IO::outl<trace>("a0: ", adc::mData[0], " a1: ", adc::mData[1], " g: ", gain , " p: ", estimator::period);
                // IO::outl<trace>("c0: ", servo_pa::value(ch_t{0}).toInt(), " dd: ", dd);
                // IO::outl<trace>("r: ", sensor::ProtocollAdapter::requests(), " b: ", sensor::StatisticProvider::mBytes);
            });
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
                pwm::duty(1300);
                pwm3::set();
                break;
            }
        }
    }
private:
    static inline uint8_t gain{1};
    static inline uint16_t dd{};
    static inline volatile State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mTelemTick;
};

using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    NVIC_EnableIRQ(TIM6_DAC_IRQn);
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
        devs::tp2::set();
        devs::tp2::reset();
    }
}

extern "C" {

void ADC1_2_IRQHandler() {
    // devs::tp::set();
    if (ADC2->ISR & ADC_ISR_EOS) {
        subSampler::isr();
    }
    ADC1->ISR = ADC_ISR_EOC;
    // devs::tp::reset();
}

void TIM6_DAC_IRQHandler() {
    TIM6->SR &= ~TIM_SR_UIF;
    // devs::nsleep::set();
    // devs::nsleepPulseWaiter::stop();
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
