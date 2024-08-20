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

#include "config.h"
#include "cmsis.h"
#include "rlestimator.h"
#include "identifier.h"
#include "gnuplot.h"
#include "plotadapter.h"
#include "fftestimator.h"
#include "subsampler.h"
#include "eeprom.h"
#include "crsf.h"
#include "fsmRun.h"
#include "fsmRL.h"
#include "fsmKM.h"
#include "toneplay.h"

using namespace std::literals::chrono_literals;

struct Storage {
    static inline void init() {
        std::memcpy(&eeprom, &eeprom_flash, sizeof(EEProm));
        // eeprom = eeprom_flash; // not working: needs volatile
    }
    static inline void reset() {
        eeprom = EEProm{};
    }

    __attribute__((__section__(".eeprom")))
    static inline const EEProm eeprom_flash{};

    static inline EEProm eeprom;
};

struct Speed {
    static inline void updateDutyFilter(const uint8_t v) {
        dutyFilter.factor(inertiaF(v));
    }
    static inline void updateTempFilter(const uint8_t v) {
        tempFilter.factor(tempF(v));
    }
    private:
    static inline float inertiaF(const uint8_t v) {
        if (v > 9) return 0.1f;
        const float f = (10 - v) * 0.1f;
        return f;
    }
    static inline float tempF(const uint8_t v) {
        if (v > 9) return 0.01f;
        const float f = (100 - (v * 11)) * 0.01f;
        return f;
    }
    public:
    static inline Dsp::ExpMean<void> dutyFilter{inertiaF(Storage::eeprom.inertia)};
    static inline Dsp::ExpMean<void> tempFilter{tempF(Storage::eeprom.temp_filter)};
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    using led = devs::ledBlinker;
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

    static inline constexpr uint16_t fSize = 1024;
    using subSampler = Dsp::SubSampler<devs, fSize>;
    using estimator = Dsp::FFTEstimator<fSize, subSampler>;

    using identifier = BdcIdentifier<devs>;
    using config = Config<estimator, subSampler, pwm>;

    using rlfsm = RLFsm<systemTimer, pwm, identifier, config, trace>;
    // using rlfsm = RLFsm<systemTimer, pwm, identifier, config, void>;

    // using kmfsm = KmFsm<systemTimer, pwm, estimator, config, trace>;
    using kmfsm = KmFsm<systemTimer, pwm, estimator, config, void>;

    using toneplay = External::TonePlayer<pwm, systemTimer, Storage, void>;

#ifdef USE_GNUPLOT
    using plot = Graphics::Gnuplot<trace, Meta::List<Adapter0<estimator>, Adapter1<estimator>, Adapter2<estimator>, Adapter3<estimator>, Adapter4<estimator>>>;
    // using plot = Graphics::Gnuplot<hsout, Meta::List<Adapter0<estimator>>>;
#endif

    static inline void init() {
        devs::init();
        config::init();
        estimator::init();
    }

    enum class State : uint8_t {Undefined = 0x00,
                                Init,
                                Check, Run, Reset,
                                PlayStartupTone,
                                PlayRunTone,
                                PlayWaitTone,
                                PlayTone,
                                PrintMeasures,
                                StartCalibrate,
                                StopCalibrate,
                                UpdateRL,
                                MeasRL,
                                MeasRot
                               };
    enum class Event : uint8_t {None,
                                Init,
                                StartCalibrate,
                                StopCalibrate
                               };

    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void resetParameter() {
        Storage::reset();
        crsfCallback::callbacks();
        crsfCallback::save();
    }

    static inline void updatePwm() {
        // config::cutoff(Storage::eeprom.cutoff_freq * 100);
        // config::subSampleFactor(Storage::eeprom.subsampling);
        // config::pwm(Storage::eeprom.subsampling * Storage::eeprom.cutoff_freq * Storage::eeprom.n_fsample);
    }

    static inline void update() {
        crsfCallback::update();
        crsfCallback::callbacks();
    }

    static inline void periodic() {
        trace::periodic();
        servo::periodic();
        sensor_uart::periodic();

        switch(mState) {
        case State::Undefined:
            break;
        case State::Init:
            break;
        case State::Check:
            break;
        case State::PlayStartupTone:
            break;
        case State::PlayRunTone:
            break;
        case State::PlayWaitTone:
            break;
        case State::PlayTone:
            break;
        case State::Run:
#ifdef USE_GNUPLOT
            plot::periodic();
#endif
            break;
        case State::Reset:
            break;
        case State::StartCalibrate:
            break;
        case State::StopCalibrate:
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

    static inline constexpr External::Tick<systemTimer> waitTicks{1000ms};
    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemTicks{50ms};

    enum class LinkStatus : uint8_t {Undefined, Connected, NotConnected};
    static inline LinkStatus mLinkStatus{LinkStatus::Undefined};

    static inline void ratePeriodic() {
        const auto oldState = mState;

        led::ratePeriodic();
        servo_pa::ratePeriodic([](const bool connected){
            const LinkStatus ls = connected ? LinkStatus::Connected : LinkStatus::NotConnected;
            if (!etl::equalStore(mLinkStatus, ls)) {
                if (connected) {
                    led::event(led::Event::Slow);
                }
                else {
                    led::event(led::Event::Fast);
                }
            }
        });
        crsfTelemetry::ratePeriodic();
        crsf_out::ratePeriodic();

        toneplay::ratePeriodic();

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
                if (Storage::eeprom.prerun_check > 0) {
                    mState = State::Check;
                }
                else {
                    mState = State::Run;
                }
            });
            break;
        case State::PlayStartupTone:
            if (toneplay::isOff()) {
                mState = State::Init;
            }
            break;
        case State::PlayRunTone:
            if (toneplay::isOff()) {
                mState = State::Run;
            }
            break;
        case State::PlayWaitTone:
            if (toneplay::isOff()) {
                const auto input = servo_pa::normalized(Storage::eeprom.crsf_channel - 1);
                if (input == 0) {
                    mState = State::PlayRunTone;
                }
            }
            break;
        case State::PlayTone:
            break;
        case State::StartCalibrate:
            mStateTick.on(waitTicks, [&]{
                mState = State::MeasRL;
            });
            break;
        case State::StopCalibrate:
            mStateTick.on(waitTicks, [&]{
                if (Storage::eeprom.prerun_check > 0) {
                    mState = State::Check;
                }
                else {
                    mState = State::Run;
                }
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
            mState = State::StopCalibrate;
            break;
        case State::Check: {
            const auto input = servo_pa::normalized(Storage::eeprom.crsf_channel - 1);
            mStateTick.on(initTicks, [&]{
                IO::outl<trace>("# ch: ", Storage::eeprom.crsf_channel, " v: ", (int16_t)input, " conn: ", (uint8_t)mLinkStatus);
            });
            if (mLinkStatus == LinkStatus::Connected) {
                if (input == 0) {
                    mState = State::PlayRunTone;
                } else {
                    mState = State::PlayWaitTone;
                }
            }
        }
            break;
        case State::Run:
            if (std::exchange(mEvent, Event::None) == Event::StartCalibrate) {
                mState = State::StartCalibrate;
            }
            mTelemTick.on(telemTicks, [&]{
                const auto input = servo_pa::normalized(Storage::eeprom.crsf_channel - 1);
                estimator::dir1(input >= 0);
                estimator::update(input);
                const uint16_t mRpm = estimator::eRpm() / Storage::eeprom.telemetry_polepairs;
                if (mRpm > 26) {
                    crsfTelemetry::rpm1(mRpm);
                }
                else {
                    crsfTelemetry::rpm1(0);
                }
                crsfTelemetry::batt(devs::adc2Voltage(subSampler::meanVoltage()) * 10);
                crsfTelemetry::curr(devs::adc2Current(subSampler::currMean()) * 10);

                const uint16_t t = Speed::tempFilter.process(Mcu::Stm::adc2Temp(adc::mData[2]));
                crsfTelemetry::temp1(t);

                const auto vf = Speed::dutyFilter.process(input);

                if (const auto b = vf.absolute(); vf >= 0) {
                    pwm::dir1();
                    pwm::duty(b);
                }
                else {
                    pwm::dir2();
                    pwm::duty(b);
                }
            });
            mStateTick.on(initTicks, [&]{
                IO::outl<trace>("# rpm: ", estimator::eRpmNoWindow() / Storage::eeprom.telemetry_polepairs);
                IO::outl<trace>("# meanADC: ", (uint16_t)subSampler::currMeanADC(), " mean: ", (uint16_t)subSampler::currMean(), " g:", subSampler::gain(), " volt: ", (uint16_t)(10.0f * devs::adc2Voltage(subSampler::meanVoltage())));
            });
            break;
        case State::Reset:
            if (std::exchange(mEvent, Event::None) == Event::Init) {
                mState = State::PlayStartupTone;
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                IO::outl<trace>("# Init");
                led::event(led::Event::Steady);
                adc::start();
                break;
            case State::PlayStartupTone:
                toneplay::event(toneplay::Event::PlayTune1);
                break;
            case State::PlayRunTone:
                toneplay::event(toneplay::Event::PlayTune3);
                break;
            case State::PlayWaitTone:
                toneplay::event(toneplay::Event::PlayTune4);
                break;
            case State::PlayTone:
                IO::outl<trace>("# PlayTone");
                pwm::setToneMode();
                pwm::duty(100);
                pwm::pwm(400);
                break;
            case State::StartCalibrate:
                IO::outl<trace>("# Start Cal");
                led::event(led::Event::Medium);
                break;
            case State::StopCalibrate:
                IO::outl<trace>("# Stop Cal");
                break;
            case State::MeasRL:
                IO::outl<trace>("# MeasRL");
                rlfsm::start();
                break;
            case State::UpdateRL: {
                IO::outl<trace>("# UpdateRL");
                const auto Rm = rlfsm::getLastRm();
                kmfsm::setRm(Rm); }
                break;
            case State::MeasRot:
                config::init();
                IO::outl<trace>("# MeasRot");
                kmfsm::start();
                break;
            case State::PrintMeasures: {
                IO::outl<trace>("# PrintMeasures");
                rlfsm::template print<trace>();
                kmfsm::template print<trace>();
                const auto Rm = rlfsm::getLastRm();
                const auto Lm = rlfsm::getLastLm();
                const auto eKm = kmfsm::getMeanEKm();
                // todo: distinct values for both directions
                Storage::eeprom.resistance = Rm;
                Storage::eeprom.inductance = Lm;
                Storage::eeprom.eKm = eKm;
                crsfCallback::update();
                crsfCallback::save();
                IO::outl<trace>("# Setting Rm: ", (uint16_t)(1000 * Rm.dir1), " Lm: ",  (uint16_t)(1000 * Lm.dir1), " eKm: ", (uint16_t)eKm.dir1);
                estimator::setRm(Rm);
                estimator::setEKm(eKm); }
                break;
            case State::Check:
                IO::outl<trace>("# Check");
                break;
            case State::Run:
                IO::outl<trace>("# Run");
                led::event(led::Event::Slow);
                IO::outl<trace>("# ekm: ", (uint16_t)(1000 * Storage::eeprom.eKm.dir1));
                config::init();
                in1::template dir<Mcu::Output>();
                in2::afunction(2);
                pwm::duty(0);
                pwm::setSingleMode();
                break;
            case State::Reset:
                IO::outl<trace>("# Reset");
                nsleep::reset();
                nsleepPulseWaiter::start();
                break;
            }
        }
    }
    static inline bool isCalibrating() {
        return mState != State::Run;
    }
    static inline bool inRLMeasuringState() {
        return mState == State::MeasRL;
    }
    static inline bool inToneState() {
        return (mState == State::PlayTone)
                || (mState == State::PlayStartupTone)
                || (mState == State::PlayRunTone)
                || (mState == State::PlayWaitTone);
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
            TIM3->SR = ~TIM_SR_CC1IF;
            devs::tp1::set();
            if (inRLMeasuringState()) {
                if (const auto rl = identifier::calculateIsr()) {
                    // IO::outl<trace>("# Rm: ", (uint16_t)(1000 * rl->Rm), " Lm: ", (uint16_t)(rl->Lm));
                    rlfsm::addMeasurement(*rl, dir);
                }
                if (dir) {
                    pwm::dir1();
                }
                else {
                    pwm::dir2();
                }
                dir = !dir;
            }
            else if (inToneState()) {
                if (dir) {
                    pwm::dir1();
                }
                else {
                    pwm::dir2();
                }
                dir = !dir;
            }
            devs::tp1::reset();
        }
        if (TIM3->SR & TIM_SR_UIF) {
            TIM3->SR = ~TIM_SR_UIF;
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
    static inline Event mEvent{Event::None};
};

struct Notifier;

struct DevsConfig {
    using storage = Storage;
    using notifier = Notifier;
    using speed = Speed;
};
using devs = Devices<ESC20, DevsConfig, Mcu::Stm::Stm32G431>;
using gfsm = GFSM<devs>;

struct Notifier {
    static inline bool startCalibrate() {
        gfsm::event(gfsm::Event::StartCalibrate);
        return true;
    }
    static inline bool abortCalibrate() {
        return true;
    }
    static inline bool isCalibrating() {
        return gfsm::isCalibrating();
    }
    static inline void updatePwm() {
        gfsm::updatePwm();
    }
    static inline void resetParameter() {
        gfsm::resetParameter();
    }
};

int main() {
    Storage::init();
    gfsm::init();
    gfsm::update();

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
    TIM6->SR = ~TIM_SR_UIF;
    devs::nsleep::set();
    devs::nsleepPulseWaiter::stop();
    gfsm::event(gfsm::Event::Init);
}

void TIM3_IRQHandler() {
    gfsm::pwmIsr();
}

void TIM4_IRQHandler() {
    if (TIM4->SR & TIM_SR_CC1IF) {
        TIM4->SR = ~TIM_SR_CC1IF;
        devs::tp2::set();
        devs::tp2::reset();
    }
}

}
