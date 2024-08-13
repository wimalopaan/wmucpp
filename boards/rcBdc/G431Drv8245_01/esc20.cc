#define USE_MCU_STM_V3
#define USE_DEVICES2

// #define USE_GNUPLOT

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
#include "identifier.h"
#include "gnuplot.h"
#include "fftestimator.h"
#include "subsampler.h"
#include "eeprom.h"
#include "crsf.h"
#include "fsmRun.h"
#include "fsmRL.h"
#include "fsmKM.h"

using namespace std::literals::chrono_literals;

struct Storage {
    static inline void init() {
        eeprom = eeprom_flash;
    }
    __attribute__((__section__(".eeprom")))
    static inline const EEProm eeprom_flash{};

    static inline EEProm eeprom;
};

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

    static inline uint16_t fCutoff = 1'000;
    static inline uint16_t fSample = 4 * fCutoff;
    static inline uint16_t subSamplingFactor = 6;
    static inline uint16_t fPwm = fSample * subSamplingFactor;

    static inline uint16_t fPwmIdentify = 200;

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

struct DevsConfig {
    using storage = Storage;
};

using devs = Devices<ESC20, DevsConfig, Mcu::Stm::Stm32G431>;

constexpr uint16_t fSize = 1024;
using subSampler = Dsp::SubSampler<devs, fSize>;
using estimator = Dsp::FFTEstimator<fSize, subSampler>;

struct Adapter0 {
    static inline const auto& data = estimator::magnitudeWeighted();
    static inline const char* const title = "weighted mag(FFT)";
};
struct Adapter1 {
    static inline const auto& data = estimator::maxWeighted;
    static inline const char* const title = "max";
};
struct Adapter2 {
    static inline const auto& data = estimator::rpmPos;
    static inline const char* const title = "est(Um,i,Rm)";
};
struct Adapter3 {
    static inline const auto& data = estimator::windowPlot;
    static inline const char* const title = "window";
};
struct Adapter4 {
    static inline const auto& data = estimator::magnitude();
    static inline const char* const title = "mag(FFT)";
};

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

    using servo = devs::servo;
    using servo_pa = devs::servo_pa;
    using crsf_out = devs::crsf_out;
    using sensor_uart = devs::sensor_uart;
    using crsfCallback= devs::crsfCallback;
    using crsfTelemetry = crsfCallback::crsfTelemetry;
    using hsout = devs::trace;

    using rlfsm = RLFsm<systemTimer, pwm, identifier, config, trace>;

    using kmfsm = KmFsm<systemTimer, pwm, estimator, config, trace>;

#ifdef USE_GNUPLOT
    using plot = Graphics::Gnuplot<hsout, Meta::List<Adapter0, Adapter1, Adapter2, Adapter3, Adapter4>>;
#endif

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
                                UpdateRL,
                                MeasRL,
                                MeasRot
                               };

    static inline Dsp::ExpMean<void> rpmEst{0.3};

    static inline void periodic() {
        trace::periodic();
        servo::periodic();
        sensor_uart::periodic();

        switch(mState) {
        case State::Undefined:
            break;
        case State::Init:
            break;
        case State::Run:
#ifdef USE_GNUPLOT
            plot::periodic();
#endif
            break;
        case State::Reset:
            break;
        case State::PrintMeasures:
            break;
        case State::MeasRL:
            break;
        case State::MeasRot:
            break;
        case State::UpdateRL:
            break;
        }
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemTicks{50ms};

    static inline void endReset() {
        adc::start();
        mState = State::Init;
    }

    static inline void ratePeriodic() {
        const auto oldState = mState;

        servo_pa::ratePeriodic([](bool){

        });
        crsfTelemetry::ratePeriodic();
        crsf_out::ratePeriodic();

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
                mState = State::UpdateRL;
            }
            break;
        case State::UpdateRL:
            mState = State::MeasRot;
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
            mTelemTick.on(telemTicks, [&]{
                estimator::update(servo_pa::normalized(0));
                const int16_t v = servo_pa::normalized(0);
                if (v >= 0) {
                    pwm::duty((v * 1640.0f) / 1000.0f);
                }
            });
            mStateTick.on(initTicks, [&]{
                // IO::outl<trace>("# ch0: ", servo_pa::mChannels[0]);
                IO::outl<trace>("# meanADC: ", (uint16_t)subSampler::currMeanADC(), " mean: ", (uint16_t)subSampler::currMean(), " g:", subSampler::gain(), " volt: ", (uint16_t)(10.0f * devs::adc2Voltage(subSampler::meanVoltage())));
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
                IO::outl<trace>("# Init");
                adc::start();
                break;
            case State::MeasRL:
                IO::outl<trace>("# MeasRL");
                rlfsm::start();
                break;
            case State::UpdateRL:
            {

                IO::outl<trace>("# UpdateRL");
                const float Rm = rlfsm::getLastRm();
                kmfsm::setRm(Rm);
            }
                break;
            case State::MeasRot:
                config::init();
                IO::outl<trace>("# MeasRot");
                kmfsm::start();
                break;
            case State::PrintMeasures:
                IO::outl<trace>("# PrintMeasures");
                rlfsm::print();
                break;
            case State::Run:
                IO::outl<trace>("# Run");
                config::init();
                in1::template dir<Mcu::Output>();
                in2::afunction(2);
                pwm::duty(0);
                pwm::setSingleMode();
                led::reset();
                break;
            case State::Reset:
                IO::outl<trace>("# Reset");
                nsleep::reset();
                nsleepPulseWaiter::start();
                break;
            }
        }
    }

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
            if ((mState == State::Run) || (mState == State::MeasRot)) {
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
                    IO::outl<trace>("# Rm: ", (uint16_t)(1000 * rl->Rm), " Lm: ", (uint16_t)(rl->Lm));
                    rlfsm::addMeasurement(*rl, dir);
                }
                if (dir) {
                    // pwm::dir()
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
    Storage::init();
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
