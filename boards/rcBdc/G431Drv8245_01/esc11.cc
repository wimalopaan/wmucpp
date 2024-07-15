#define USE_MCU_STM_V3
#define NDEBUG

namespace Mcu::Stm {
inline namespace V3{}
}

#include "devices.h"

#include <chrono>
#include <cassert>
#include <numbers>

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

    // static inline uint16_t fCutoff = 750;
    static inline uint16_t fCutoff = 1'000;
    static inline uint16_t fSample = 4 * fCutoff;
    static inline uint16_t subSamplingFactor = 6;
    static inline uint16_t fPwm = fSample * subSamplingFactor;

    // static inline uint16_t fPwm = 20'000;
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

inline constexpr float adc2Voltage(const auto a) {
    constexpr float r1 = 91'000;
    constexpr float r2 = 10'000;
    constexpr float vref = 3.3f;
    constexpr float max = 4095.0f;

    return (a * vref) * (r1 + r2) / (max * r2);
}
inline constexpr float adc2Current(const auto i) {
    constexpr float iFactor = 6150;
    constexpr float ri = 1'000;
    constexpr float vref = 3.3f;
    constexpr float max = 4095.0f;

    return (i * vref * iFactor) / (max * ri);
}

template<uint16_t Size, typename Source>
struct FFTEstimator {
    static inline void init() {
        arm_status status = arm_rfft_fast_init_f32(&fftInstance, Size);
        for(uint16_t i = 0; i < (2 * width); ++i) {
            d3[i].second = window[i];
        }
    }
    static inline void update(const auto p) {
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

        if (p) {
            const uint16_t pmax = 300; // fftindex T60S
            // const uint16_t pmax = 380; // fftindex 500e
            const float Ubatt = adc2Voltage(Source::mMeanVoltage.value());

            const uint16_t px = p.toInt() + 1000; // 0 ... 2000
            const uint16_t mid = (pmax * px) / 2000;
            const uint16_t a = std::max(mid - width, 1);
            const uint16_t b = std::min(mid + width, Size/2 - 1);
            for(uint16_t i = 0; i < Size / 2; ++i) {
                if (i < minIndex) {
                    magnitude2[i] = 0;
                    magnitude[i] = 0;
                }
                else {
                    magnitude2[i] = magnitude[i];
                    magnitude[i] *= wf(i, mid);
                }
            }
            uint16_t offset = minIndex + 1;
            arm_max_f32(&magnitude[offset], Size / 2 - offset, &maxValue, &maxIndex2);
            maxIndex2 += offset;

            for(uint16_t i = 0; i < Size / 2; ++i) {
                magnitude2[i] = std::min(magnitude2[i], maxValue * 5.0f);
            }

            d1[0].first = std::max((int)maxIndex2 - 10, 0);
            d1[0].second = maxValue * 0.9;
            d1[1].first = maxIndex2;
            d1[1].second = maxValue;
            d1[2].first = std::min((int)maxIndex2 + 10, Size/2);
            d1[2].second = maxValue * 0.9;

            const float curr = adc2Current(Source::mMean2.value());
            // const float curr = Source::mMean2.value() / 201.0f;

            const float udiff = curr * 0.83f; // TS60
            // const float udiff = curr * 0.1f; // Torque 900
            const float umotor = ((Ubatt * px) / 2000 - udiff);

            const float rpm = pmax * umotor / Ubatt;

            d2[0].first = std::max((int)rpm - 10, 0);
            d2[0].second = maxValue * 0.9;
            d2[1].first = rpm;
            d2[1].second = maxValue;
            d2[2].first = std::min((int)rpm + 10, Size/2);
            d2[2].second = maxValue * 0.9;

            d3.clear();
            for(uint16_t i = a; i < b; ++i) {
                std::pair<uint16_t, float> c;
                c.first = i;
                c.second = maxValue * wf(i, rpm);
                d3.push_back(c);
            }
        }
    }
// private:
    static inline uint32_t minIndex{0};
    static inline float maxValue{};
    static inline uint32_t maxIndex{};
    static inline uint32_t maxIndex2{};
    static inline arm_rfft_fast_instance_f32 fftInstance{};
    static inline std::array<float, Size> samples{};
    static inline std::array<float, Size> fft{};
    static inline std::array<float, Size / 2> magnitude{};
    static inline std::array<float, Size / 2> magnitude2{};
    public:
    static inline std::array<std::pair<uint16_t, float>, 3> d1;
    static inline std::array<std::pair<uint16_t, float>, 3> d2;

    static inline constexpr uint16_t width = 75;
    static inline constexpr auto window = []{
        std::array<float, 2 * width> w{};
        for(uint16_t i = 0; i < (2 * width); ++i) {
            const float c = cos(std::numbers::pi_v<float> * (float)(i - width) / (2 * width));
            w[i] = c * c;
        }
        return w;
    }();
    static inline constexpr float wf(const uint16_t index, const uint16_t mid) {
        const int16_t i = (index - mid) + width;
        if (i < 0) {
            return 0;
        }
        else if (i > (2 * width)) {
            return 0;
        }
        return window[i];
    }
    static inline etl::FixedVector<std::pair<uint16_t, float>, 2 * width> d3;
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
        const float x = iirFilter.process(adc::mData[0]);
        mMeanVoltage.process(adc::mData[1]);

        sampleCounter += 1;
        if (sampleCounter == factor) {
            sampleCounter = 0;
            const float xg = x / gainFactor[gain];
            data.push_back(xg);

            mMean.process(x); // ADC overflow
            mMean2.process(xg); // real value

            if (mMean.value() > 4095.0f * (2.0f / 3.0f)) {
                if (gain > 0) {
                    gain -= 1;
                    pga::gain(gain);
                    mMean.set(mMean.value() / 2);
                }
            }
            else if (mMean.value() < 4095.0f * (1.0f / 3.0f)) {
                if (gain < 6) {
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
    static inline constexpr std::array<uint8_t, 7> gainFactor{1, 2, 4, 8, 16, 32, 64};
    static inline Dsp::Butterworth::LowPass<6, volatile float> iirFilter;
    static inline Dsp::ExpMean<void> mMean{0.001};
    static inline Dsp::ExpMean<void> mMean2{0.01};
    static inline Dsp::ExpMean<void> mMeanVoltage{0.0001};
};

using devs = Devices<ESC11, void, Mcu::Stm::Stm32G431>;


constexpr uint16_t fSize = 1024;
using subSampler = SubSampler<devs, fSize>;
using estimator = FFTEstimator<fSize, subSampler>;

struct Adapter0 {
    static inline const auto& data = estimator::magnitude;
    static inline const char* const title = "weighted mag(FFT)";
};
struct Adapter1 {
    static inline const auto& data = estimator::d1;
    static inline const char* const title = "max";
};
struct Adapter2 {
    static inline const auto& data = estimator::d2;
    static inline const char* const title = "est(Um,i,Rm)";
};
struct Adapter3 {
    static inline const auto& data = estimator::d3;
    static inline const char* const title = "window";
};
struct Adapter4 {
    static inline const auto& data = estimator::magnitude2;
    static inline const char* const title = "mag(FFT)";
};

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

    using plot = Graphics::Gnuplot<hsout, Meta::List<Adapter0, Adapter1, Adapter2, Adapter3, Adapter4>>;

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

            subSampler::whenValueAvailable([](const float v){
                tp2::set();
                tp2::reset();
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
                estimator::update(servo_pa::normalized(0));
                // tp2::reset();

                const auto v = servo_pa::normalized(0);
                const uint16_t d = 1640 * ((v.toInt() + 1000.0f) / 2000.0f);
                pwm::duty(d);

                const auto t = servo_pa::normalized(1);
                const float tv = ((t.toInt() + 1000.0f) / 2000.0f);
                pwm::trigger(tv);
            });

            mStateTick.on(initTicks, []{
                // IO::outl<trace>("ma: ", estimator::maxIndex, " mi:", estimator::minIndex, " g:", subSampler::gain, " exp:", (uint16_t)subSampler::mMean.value(), " a:", (uint16_t)adc::mData[0]);
                // IO::outl<trace>("ma: ", estimator::maxIndex, " ma2: ", estimator::maxIndex2, " mean: ", (uint16_t)subSampler::mMean.value(), " g:", subSampler::gain);
                IO::outl<trace>("mean: ", (uint16_t)subSampler::mMean.value(), " mean2: ", (uint16_t)subSampler::mMean2.value(), " g:", subSampler::gain, " volt: ", (uint16_t)(10.0f * adc2Voltage(subSampler::mMeanVoltage.value())));
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

void TIM3_IRQHandler() {
    devs::tp1::set();
    TIM3->SR &= ~TIM_SR_CC3IF;
    devs::tp1::reset();
}

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
