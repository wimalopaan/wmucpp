#define USE_MCU_STM_V3
#define NDEBUG

namespace Mcu::Stm {
    inline namespace V3{}
}

#include "devices.h"

#include <chrono>
#include <cassert>
#include <numbers>

#include <etl/circularbuffer.h>

#include "cmsis.h"
#include "rlestimator.h"
#include "gnuplot.h"
#include "fftestimator.h"
#include "subsampler.h"

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

    static inline uint16_t fPwmIdentify = 200;
    // static inline uint16_t maxRpm = 10'000;

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

using devs = Devices<ESC15, void, Mcu::Stm::Stm32G431>;

constexpr uint16_t fSize = 1024;
using subSampler = Dsp::SubSampler<devs, fSize>;
using estimator = Dsp::FFTEstimator<fSize, subSampler>;

struct Adapter0 {
    static inline const auto& data = estimator::magnitude;
    static inline const char* const title = "weighted mag(FFT)";
};
struct Adapter1 {
    static inline const auto& data = estimator::mMaxWeighted;
    static inline const char* const title = "max";
};
struct Adapter2 {
    static inline const auto& data = estimator::mRpmPos;
    static inline const char* const title = "est(Um,i,Rm)";
};
struct Adapter3 {
    static inline const auto& data = estimator::window;
    static inline const char* const title = "window";
};
struct Adapter4 {
    static inline const auto& data = estimator::magnitudeWeighted;
    static inline const char* const title = "mag(FFT)";
};

template<typename T = float>
struct RL {
    T Rm;
    T Lm;
    template<typename U>
    void operator+=(const RL<U> rhs) {
        Rm += rhs.Rm;
        Lm += rhs.Lm;
    }
    template<typename U>
    void operator=(const RL<U> rhs) {
        Rm = rhs.Rm;
        Lm = rhs.Lm;
    }
    const RL& operator/=(const float d) {
        Rm /= d;
        Lm /= d;
        return *this;
    }
};

template<typename T = float, uint8_t Size = 20>
struct RLMeasurements {
    void operator+=(const std::pair<RL<>, RL<>>& m) {
        if (step < Size) {
            meanRL_dir1[step] = m.first;
            meanRL_dir2[step] = m.second;
            ++step;
        }
    }
    uint16_t step{};
    std::array<RL<T>, Size> meanRL_dir1{};
    std::array<RL<T>, Size> meanRL_dir2{};
};

template<typename Devices, typename Out = void>
struct BdcIdentifier {
    using devs = Devices;
    using adc = devs::adc;
    using pga = devs::pga;

    enum class State : uint8_t {Wait, Measure, Measure2};

    static inline void reset() {
        pga::gain(0); // follower-mode
        wait();
    }

    static inline void wait() {
        mState = State::Wait;
    }

    static inline void startMeasure() {
        if (mState == State::Wait) {
            voltageMean = 0.0f;
            sampleCount = 0;
            mState = State::Measure;
        }
    }
    static inline void sampleIsr() {
        lastAdc = adc::mData[0];
        const float current = devs::adc2Current(lastAdc);
        if (mState == State::Measure) {
            voltageMean += devs::adc2Voltage(adc::mData[1]);
            if (current > 0) { // warten bis Werte kommen
                devs::tp2::set();
                rle = Statistics::RLEstimator<volatile float>{1.0f, 0.0f, current};
                rle.process(current);
                sampleCount = sampleCount + 1;
                mState = State::Measure2;
                devs::tp2::reset();
            }
        }
        else if (mState == State::Measure2) {
            devs::tp2::set();
            voltageMean += devs::adc2Voltage(adc::mData[1]);
            rle.process(current);
            sampleCount = sampleCount + 1;
            devs::tp2::reset();
        }
    }
    static inline std::optional<RL<>> calculateIsr() {
        if ((mState == State::Measure2) && (sampleCount > 0)) {
            mState = State::Wait;
            maxEst.process(lastAdc);
            voltageMean /= sampleCount;
            return evaluate(voltageMean);
        }
        return {};
    }

    private:
    static inline std::optional<RL<>> evaluate(const float Um) {
        if (const auto ab = rle.compute()) {
            const float a = -ab->second / ab->first;
            const float b = ab->first;
            const float Rm = Um / a;
            const float Lm = (-a / b) / Um;
            if constexpr (!std::is_same_v<Out, void>) {
                IO::outl<Out>("Um: ", (uint16_t)(1000 * Um), " Rm: ", (uint16_t)(1000 * Rm), " Lm: ", (uint16_t)(1000 * Lm));
            }
            return RL<>{Rm, Lm};
        }
        return {};
    }

    static inline Statistics::RLEstimator<volatile float> rle{1.0f, 0.0, 0.0};
    static inline volatile uint16_t sampleCount = 0;
    static inline volatile uint16_t lastAdc = 0;
    static inline volatile float voltageMean{};
    static inline volatile State mState{State::Wait};
    public:
    static inline volatile Dsp::ExpMean<void> maxEst{0.1};
};

// using identifier = BdcIdentifier<devs, devs::hsout>;
using identifier = BdcIdentifier<devs>;

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

    template<typename> struct RLFsm;
    using rlfsm = RLFsm<trace>;

    template<typename> struct KmFsm;
    using kmfsm = KmFsm<trace>;

    using plot = Graphics::Gnuplot<hsout, Meta::List<Adapter0, Adapter1, Adapter2, Adapter3, Adapter4>>;

    static inline void init() {
        devs::init();
        led::set();
        config::init();
        estimator::init();
    }

    enum class State : uint8_t {Undefined = 0x00,
                                Init,
                                Run, Reset,
                                PrintMeasures,
                                MeasRL,
                                MeasRot
                               };

    static inline Dsp::ExpMean<void> rpmEst{0.3};

    static inline void periodic() {
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
            break;
        case State::Reset:
            break;
        case State::PrintMeasures:
            break;
        case State::MeasRL:
            break;
        case State::MeasRot:
            break;
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
                mState = State::MeasRL;
            });
            break;
        case State::MeasRL:
            if (rlfsm::process()) {
                mState = State::MeasRot;
            }
            break;
        case State::MeasRot:
            if (kmfsm::process()) {
                mState = State::PrintMeasures;
            }
            break;
        case State::PrintMeasures:
            mState = State::Run;
            break;
        case State::Run:
            mTelemTick.on(telemTicks, []{
                estimator::update(servo_pa::normalized(0));

                const auto v = servo_pa::normalized(0);

                pwm::duty(v);
            });
            mStateTick.on(initTicks, []{
                IO::outl<trace>("meanADC: ", (uint16_t)subSampler::mCurrMeanADC.value(), " mean: ", (uint16_t)subSampler::mCurrMean.value(), " g:", subSampler::gainIndex, " volt: ", (uint16_t)(10.0f * devs::adc2Voltage(subSampler::mMeanVoltage.value())));
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
                IO::outl<trace>("Init");
                adc::start();
                break;
            case State::MeasRL:
                IO::outl<trace>("MeasRL");
                rlfsm::start();
                break;
            case State::MeasRot:
                IO::outl<trace>("MeasRot");
                kmfsm::start();
                break;
            case State::PrintMeasures:
                IO::outl<trace>("PrintMeasures");
                rlfsm::print();
                break;
            case State::Run:
                IO::outl<trace>("Run");
                config::init();
                in1::template dir<Mcu::Output>();
                in2::afunction(2);
                pwm::duty(0);
                pwm::setSingleMode();
                led::reset();
                break;
            case State::Reset:
                IO::outl<trace>("Reset");
                nsleep::reset();
                nsleepPulseWaiter::start();
                break;
            }
        }
    }

    struct RunFsm {
        enum class State : uint8_t {Idle, Stop, Forward, Backward};
        enum class Event : uint8_t {NoEvent, Start};

        static inline void start() {
            if (mState == State::Idle) {
                mLastEvent = Event::Start;
            }
        }
        static inline bool process() {
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Idle:
                if (std::exchange(mLastEvent, Event::NoEvent) == Event::Start) {
                    mState = State::Stop;
                }
                break;
            case State::Stop:
                break;
            case State::Forward:
                break;
            case State::Backward:
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Idle:
                    break;
                case State::Stop:
                    break;
                case State::Forward:
                    break;
                case State::Backward:
                    break;
                }
            }
            return false;
        }
        private:
        static inline Event mLastEvent = Event::NoEvent;
        static inline State mState = State::Idle;
        static inline External::Tick<systemTimer> mStateTick;
    };

    template<typename Out = void>
    struct RLFsm {
        enum class State : uint8_t {Idle, Start, Inc, Meas, Stop};
        enum class Event : uint8_t {NoEvent, Start};

        static inline constexpr External::Tick<systemTimer> measTicks{300ms};

        static inline void start() {
            if (mState == State::Idle) {
                mLastEvent = Event::Start;
            }
        }
        static inline bool process() {
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Idle:
                if (std::exchange(mLastEvent, Event::NoEvent) == Event::Start) {
                    mState = State::Start;
                }
                break;
            case State::Start:
                mStateTick.on(measTicks, []{
                    mState = State::Inc;
                });
                break;
            case State::Inc:
                if (measureDuty > (1640 / 2)) {
                    mState = State::Stop;
                }
                else {
                    mState = State::Meas;
                }
                break;
            case State::Meas:
                // wenn Sättigung erreicht
                if (identifier::maxEst.value() > 4000) {
                    mState = State::Stop;
                }
                mStateTick.on(measTicks, []{
                    mState = State::Inc;
                });
                break;
            case State::Stop:
                mState = State::Idle;
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Idle:
                    return true;
                    break;
                case State::Start:
                    IO::outl<trace>("MeasStart");
                    measurements = RLMeasurements<volatile float>{};
                    measureDuty = 0;
                    pwm::duty(measureDuty);
                    pwm::setMultiMode();
                    pwm::pwm(config::fPwmIdentify);
                    identifier::wait();
                    break;
                case State::Inc:
                    IO::outl<trace>("MeasInc");
                    // save RL values
                    if ((actualMeasCount_dir1 > minMeasuresPerLevel) && (actualMeasCount_dir2 > minMeasuresPerLevel)) {
                        RL<> m1;
                        uint16_t c1;
                        RL<> m2;
                        uint16_t c2;
                        __disable_irq();
                        m1 = actualMeas_dir1;
                        m2 = actualMeas_dir2;
                        c1 = actualMeasCount_dir1;
                        c2 = actualMeasCount_dir2;
                        actualMeasCount_dir1 = 0;
                        actualMeasCount_dir2 = 0;
                        actualMeas_dir1 = RL<>{};
                        actualMeas_dir2 = RL<>{};
                        __enable_irq();
                        m1 /= c1;
                        m2 /= c2;
                        measurements += {m1, m2};
                    }
                    measureDuty += 100;
                    pwm::duty(measureDuty);
                    break;
                case State::Meas:
                    IO::outl<trace>("MeasMeas");
                    break;
                case State::Stop:
                    IO::outl<trace>("MeasStop");
                    identifier::reset();
                    measureDuty = 0;
                    pwm::duty(measureDuty);
                    break;
                }
            }
            return false;
        }
        static inline void addMeasurement(const RL<> rl, const bool dir) {
            if (dir) {
                actualMeas_dir1 += rl;
                actualMeasCount_dir1 += 1;
            }
            else {
                actualMeas_dir2 += rl;
                actualMeasCount_dir2 += 1;
            }
        }
        static inline void print() {
            for(uint8_t i = 0; i < measurements.step; ++i) {
                IO::outl<Out>("step: ", i, " Rm1: ", (uint16_t)(1000 * measurements.meanRL_dir1[i].Rm), " Rm2: ", (uint16_t)(1000 * measurements.meanRL_dir2[i].Rm));
                IO::outl<Out>("step: ", i, " Lm1: ", (uint16_t)(1000 * measurements.meanRL_dir1[i].Lm), " Lm2: ", (uint16_t)(1000 * measurements.meanRL_dir2[i].Lm));
            }
        }
        private:
        static inline constexpr uint16_t minMeasuresPerLevel = 10;
        static inline RL<volatile float> actualMeas_dir1{};
        static inline volatile uint16_t actualMeasCount_dir1{0};
        static inline RL<volatile float> actualMeas_dir2{};
        static inline volatile uint16_t actualMeasCount_dir2{0};
        static inline RLMeasurements<volatile float> measurements;

        static inline Event mLastEvent = Event::NoEvent;
        static inline uint16_t measureDuty{};
        static inline State mState = State::Idle;
        static inline External::Tick<systemTimer> mStateTick;
    };

    template<typename Out = void>
    struct KmFsm {
        enum class State : uint8_t {Idle, Start, Inc, Meas, Stop};
        enum class Event : uint8_t {NoEvent, Start};

        static inline constexpr External::Tick<systemTimer> measTicks{300ms};

        static inline void start() {
            if (mState == State::Idle) {
                mLastEvent = Event::Start;
            }
        }

        static inline bool process() {
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Idle:
                if (std::exchange(mLastEvent, Event::NoEvent) == Event::Start) {
                    mState = State::Start;
                }
                break;
                case State::Start:
                    mStateTick.on(measTicks, []{
                        mState = State::Inc;
                    });
                    break;
                case State::Inc:
                    if (measureDuty > (1640 / 2)) {
                        mState = State::Stop;
                    }
                    else {
                        mState = State::Meas;
                    }
                    break;
                case State::Meas:
                    mStateTick.on(measTicks, []{
                        mState = State::Inc;
                    });
                    break;
                case State::Stop:
                    mState = State::Idle;
                    break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Idle:
                    return true;
                    break;
                case State::Start:
                    IO::outl<trace>("MeasRotStart");
                    config::init();
                    in1::template dir<Mcu::Output>();
                    in2::afunction(2);
                    measureDuty = 0;
                    pwm::duty(measureDuty);
                    pwm::setSingleMode();
                    break;
                case State::Inc:
                    IO::outl<trace>("MeasRotInc");
                    measureDuty += 100;
                    pwm::duty(measureDuty);
                    break;
                case State::Meas:
                    IO::outl<trace>("MeasRotMeas");
                    break;
                case State::Stop:
                    IO::outl<trace>("MeasRotStop");
                    measureDuty = 0;
                    pwm::duty(measureDuty);
                    break;
                }
            }
            return false;
        }

        private:
        static inline Event mLastEvent = Event::NoEvent;
        static inline uint16_t measureDuty{};
        static inline State mState = State::Idle;
        static inline External::Tick<systemTimer> mStateTick;
    };

    static inline bool inRLMeasuringState() {
        return mState == State::MeasRL;
    }

    static inline void adcIsr() {
        devs::tp0::set();
        if (ADC1->ISR & ADC_ISR_EOC) {
            ADC1->ISR = ADC_ISR_EOC;
        }
        if (ADC1->ISR & ADC_ISR_EOS) {
            ADC1->ISR = ADC_ISR_EOS;
            if (mState == State::Run) {
                subSampler::isr();
            }
            else if (inRLMeasuringState()) {
                identifier::sampleIsr();
            }
        }
        devs::tp0::reset();
    }
    static inline void pwmIsr(){
        volatile static bool dir = false;
        if (TIM3->SR & TIM_SR_CC1IF) {
            TIM3->SR &= ~TIM_SR_CC1IF;
            devs::tp1::set();
            if (inRLMeasuringState()) {
                if (const auto rl = identifier::calculateIsr()) {
                    IO::outl<trace>("Rm: ", (uint16_t)(1000 * rl->Rm), " Lm: ", (uint16_t)(rl->Lm));
                    rlfsm::addMeasurement(*rl, dir);
                }
                if (dir) {
                    devs::in1::template dir<Mcu::Output>();
                    devs::in2::afunction(2);
                }
                else {
                    devs::in2::template dir<Mcu::Output>();
                    devs::in1::afunction(2);
                }
                dir = !dir;
            }
            devs::tp1::reset();
        }
        if (TIM3->SR & TIM_SR_UIF) {
            TIM3->SR &= ~TIM_SR_UIF;
            if (inRLMeasuringState()) {
                devs::tp1::set();
                identifier::startMeasure();
                devs::tp1::reset();
            }
        }
    }
    private:
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
    NVIC_EnableIRQ(TIM4_IRQn);
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
    gfsm::adcIsr();
}

void TIM6_DAC_IRQHandler() {
    TIM6->SR &= ~TIM_SR_UIF;
    devs::nsleep::set();
    devs::nsleepPulseWaiter::stop();
    gfsm::endReset();
}

void TIM3_IRQHandler() {
    gfsm::pwmIsr();
}

void TIM4_IRQHandler() {
    if (TIM4->SR & TIM_SR_CC1IF) {
        TIM4->SR &= ~TIM_SR_CC1IF;
        devs::tp2::set();
        devs::tp2::reset();
    }
}

}
