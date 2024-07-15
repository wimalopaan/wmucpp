#define USE_MCU_STM_V3
#define NDEBUG

namespace Mcu::Stm {
    inline namespace V3{}
}

#include "devices.h"

#include <chrono>
#include <cassert>

#include "cmsis.h"
#include "gnuplot.h"

using namespace std::literals::chrono_literals;

namespace detail {
    template<typename T> concept ConceptSubSFactor= requires(T) { T::subSampleFactor(0u); };
    template<typename T> struct hasSubSFactor: std::integral_constant<bool, false> {};
    template<ConceptSubSFactor T> struct hasSubSFactor<T> : std::integral_constant<bool, true> {};

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
    using ssubfListeners = Meta::filter<detail::hasSubSFactor, listeners>;
    // using pwmListeners::_;
    // using cutoffListeners::_;

    static inline uint16_t fCutoff = 750;
    // static inline uint16_t fCutoff = 1'500;
    static inline uint16_t fSample = 4 * fCutoff;
    static inline uint16_t subSamplingFactor = 6;
    // static inline uint16_t fPwm = fSample * subSamplingFactor;

    static inline uint16_t fPwm = 200;
    // static inline uint16_t fPwm = 32'000;
    // static inline uint16_t maxRpm = 10'000;

    // static inline void servo(const uint16_t s) {

    // }

    static inline void init() {
        // (Listener::init(), ...);
        pwm(fPwm);
        cutoff(fCutoff);
        subSampleFactor(subSamplingFactor);
    }
    static inline void subSampleFactor(const uint16_t f) {
        subSamplingFactor = f;
        [&]<typename... L>(Meta::List<L...>){
            (L::subSampleFactor(f), ...);
        }(ssubfListeners{});
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

// struct Estimator {
//     static inline Dsp::HystereseThreshold<void> th{0.001};
//     static inline Dsp::Butterworth::LowPass<6> iirFilter;

//     static inline void cutoff(const uint16_t f) {
//         iirFilter.fc(f);
//     }

//     static inline void pwm(const uint16_t f) {
//         iirFilter.fs(f);
//         // modify th
//     }

//     static inline void process(const float v) {
//         const auto x = iirFilter.process(v);
//         v1 = x;
//         const bool t = th.process(x);

//         // todo: mehrere Perioden, da Perioden teilw. unterschiedlich lang
//         ++pCounter;
//         if (!last && t) { // rising
//             period = pCounter;
//             pCounter = 0;
//         }
//         last = t;
//     }
//     static inline bool last;
//     static inline uint32_t pCounter;
//     static inline uint32_t period;
//     static inline float v1;
// };

template<uint16_t Size, typename Source>
struct FFTEstimator {
    static inline void init() {
        arm_status status = arm_rfft_fast_init_f32(&fftInstance, Size);
    }
    static inline void update() {
        __disable_irq();
        std::copy(std::begin(Source::data.data()), std::end(Source::data.data()), &samples[0]);
        __enable_irq();

        arm_rfft_fast_f32(&fftInstance, &samples[0], &fft[0], 0); // fft[0] (real) : dc-offset
        arm_cmplx_mag_f32(&fft[0], &magnitude[0], Size / 2); // interleaved (real, complex) input, normal output

        float minValue = magnitude[0];
        minIndex = 0;
        for(uint16_t i = 1; i < 100; ++i) {
            if (magnitude[i] < minValue) {
                minValue = magnitude[i];
                minIndex = i;
            }
            else {
                break;
            }
        }

        uint16_t offset = minIndex + 1;
        float maxValue{};
        arm_max_f32(&magnitude[offset], Size / 2 - offset, &maxValue, &maxIndex);
        maxIndex += offset;
    }


    // private:
    static inline uint32_t minIndex{0};
    static inline float maxValue{};
    static inline uint32_t maxIndex{};
    static inline arm_rfft_fast_instance_f32 fftInstance{};
    static inline std::array<float, Size> samples{};
    static inline std::array<float, Size> fft{};
    static inline std::array<float, Size / 2> magnitude{};
    public:
    static inline const auto& data = magnitude;
};

template<typename Devices, uint16_t Size = 2048>
struct SubSampler {
    using devs = Devices;
    using adc = devs::adc;
    using pga = devs::pga;

    using tp2 = devs::tp2;
    static inline void cutoff(const uint16_t f) {
        iirFilter.fc(f);
    }
    static inline void pwm(const uint16_t f) {
        iirFilter.fs(f);
    }
    static inline void subSampleFactor(const uint16_t f) {
        factor = f;
    }
    static inline void isr() {
        const auto x = iirFilter.process(adc::mData[0]);
        sampleCounter += 1;
        if (sampleCounter == factor) {
            sampleCounter = 0;
            const float xg = x / gainFactor[gain];
            data.push_back(xg);

            mMean.process(x);

            if (mMean.value() > 4095.0f * (2.0f / 3.0f)) {
                if (gain > 0) {
                    gain -= 1;
                    pga::gain(gain);
                    mMean.set(mMean.value() / 2);
                }
            }
            else if (mMean.value() < 4095.0f * (1.0f / 3.0f)) {
                if (gain < 5) {
                    gain += 1;
                    pga::gain(gain);
                    mMean.set(mMean.value() * 2);
                }
            }
        }
    }
    static inline void whenValueAvailable(auto f) {
        bool flag = false;
        float v;
        __disable_irq();
        if (!data.empty()) {
            flag = true;
            data.pop_front(v);
        }
        __enable_irq();
        if (flag) {
            f(v);
        }
    }
    // private:
    static inline etl::FiFo<volatile float, Size> data;
    static inline volatile uint16_t sampleCounter{0};
    static inline volatile uint16_t factor{10};
    static inline volatile uint8_t gain{0};
    static inline constexpr std::array<uint8_t, 6> gainFactor{2, 4, 8, 16, 32, 64};
    static inline Dsp::Butterworth::LowPass<6, volatile float> iirFilter;
    // static inline Dsp::ExpMax<void> mMax{0.0001};
    static inline Dsp::ExpMean<void> mMean{0.001};

};

using devs = Devices<ESC12, void, Mcu::Stm::Stm32G431>;


constexpr uint16_t fSize = 1024;
using subSampler = SubSampler<devs, fSize>;
using estimator = FFTEstimator<fSize, subSampler>;

using config = Config<estimator, subSampler, devs::pwm>;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    using led = devs::led;
    using tp2 = devs::tp2;
    using fault = devs::fault;

    using pwm = devs::pwm;
    using nsleep = devs::nsleep;
    using nsleepPulseWaiter = devs::nsleepPulseWaiter;
    using in1 = devs::in1;
    using in2 = devs::in2;

    using adc = devs::adc;
    using adcDmaChannel = devs::adcDmaChannel;
    using pga = devs::pga;

    // using dac = devs::dac;

    using servo = devs::servo;
    using servo_pa = devs::servo_pa;
    // using sensor = devs::ibus_sensor;
    using hsout = devs::hsout;

    using plot = Graphics::Gnuplot<hsout, Meta::List<estimator>>;

    static inline void init() {
        devs::init();
        led::set();
        config::init();
        estimator::init();
    }

    enum class State : uint8_t {Undefined, Init, Run, Reset};

    static inline Dsp::ExpMean<void> rpmEst{0.3};

    static inline void periodic() {
        // devs::tp::set();
        // devs::tp::reset();
        trace::periodic();

        servo::periodic();
        // sensor::periodic();
        hsout::periodic();

        switch(mState) {
        case State::Undefined:
            break;
        case State::Init:
            break;
        case State::Run:
            plot::periodic();
#if 0
        {
            if (hsout::isTxQueueEmpty()) {
                static uint32_t c = 0;
                if (c == 0) {
                    IO::outl<hsout>("set title 'ESC'");
                    IO::outl<hsout>("plot '-' w lines");
                    IO::outl<hsout>(c, " ", 0);
                }
                else {
                    IO::outl<hsout>(c, " ", (uint32_t)(estimator::magnitude[c]));
                }
                ++c;
                if (c == (fSize / 2)) {
                    c = 0;
                    IO::outl<hsout>("e");
                }
            }
        }
#endif
            subSampler::whenValueAvailable([](const float v){
                static uint32_t c = 0;
                tp2::set();
                uint16_t vv = v * 10;
                if (c == 0) {
                    // IO::outl<hsout>("set title 'ESC'");
                    // IO::outl<hsout>("plot '-' w lines");
                }
                // IO::outl<hsout>(c++, " ", vv);
                tp2::reset();
                if (c == 2048) {
                    c = 0;
                    // IO::outl<hsout>("e");
                }
            });

            // adc::whenSequenceComplete([]{
            //     tp2::set();
            //     const float c = adc::mData[0];
            //     // estimator::process(c);

            //     // dac::set2(std::min(estimator::v1, 4095.0f));
            //     // dac::set2(adc::mData[0]);
            //     // dac::set2(std::min(estimator::th.mMean.value(), 4095.0f));
            //     tp2::reset();
            // });
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
        // sensor::ratePeriodic();

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
                // tp2::set();
                estimator::update();
                // tp2::reset();

                const auto v = servo_pa::normalized(0);
                const uint16_t d = 1640 * ((v.toInt() + 1000.0f) / 2000.0f);
                pwm::duty(d);

                const auto t = servo_pa::normalized(1);
                const float tv = ((t.toInt() + 1000.0f) / 2000.0f);
                pwm::trigger(tv);
            });

            mStateTick.on(initTicks, []{
                // IO::outl<hsout>("ma: ", estimator::maxIndex, " mi:", estimator::minIndex, " g:", subSampler::gain, " exp:", (uint16_t)subSampler::mMean.value(), " a:", (uint16_t)adc::mData[0]);
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
                //in1::afunction(2);
                in2::afunction(2);
                pwm::duty(0);
                pwm::setSingleMode();
                // pwm::setMultiMode();
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
    // static inline uint8_t gain{1};
    static inline uint16_t dd{};
    static inline volatile State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mTelemTick;
};

using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
    // NVIC_EnableIRQ(TIM4_IRQn);
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
    if (ADC1->ISR & ADC_ISR_EOC) {
        ADC1->ISR = ADC_ISR_EOC;
        // devs::tp1::set();
        // devs::tp1::reset();
    }
    if (ADC1->ISR & ADC_ISR_EOS) {
        ADC1->ISR = ADC_ISR_EOS;
        devs::tp0::set();
        subSampler::isr();
        devs::tp0::reset();
    }
}

void TIM6_DAC_IRQHandler() {
    TIM6->SR &= ~TIM_SR_UIF;
    devs::nsleep::set();
    devs::nsleepPulseWaiter::stop();
    gfsm::endReset();
}

#if 0
// todo: besser CC2
// in die Klasse verlagern
void TIM3_IRQHandler() {
    volatile static bool c = false;
    devs::tp1::set();
    TIM3->SR &= ~TIM_SR_CC3IF;
    devs::tp1::reset();
    if (c) {
        devs::in1::template dir<Mcu::Output>();
        devs::in2::afunction(2);
    }
    else {
        devs::in2::template dir<Mcu::Output>();
        devs::in1::afunction(2);
    }
    c = !c;
}
#endif

void TIM3_IRQHandler() {
    volatile static bool c = false;
    devs::tp1::set();
    TIM3->SR &= ~TIM_SR_UIF;
    devs::tp1::reset();
    if (c) {
        devs::in1::template dir<Mcu::Output>();
        devs::in2::afunction(2);
    }
    else {
        devs::in2::template dir<Mcu::Output>();
        devs::in1::afunction(2);
    }
    c = !c;
}

// void TIM4_IRQHandler() {
//     devs::tp0::set();
//     TIM4->SR &= ~TIM_SR_UIF;
//     TIM4->SR &= ~TIM_SR_CC1IF;
//     devs::tp0::reset();
// }

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
