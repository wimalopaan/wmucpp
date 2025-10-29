#define USE_MCU_STM_V3
#define USE_DEVICES2

// #define TEST_C1

//#define USE_GNUPLOT

#define CRSF_MODULE_NAME "WM-BDC-32-L(50A)"
#define DEFAULT_POLE_PAIRS 12
#define DEFAULT_INERTIA 9
#define DEFAULT_RL_CALIB_PWM 0

#define NDEBUG 

namespace Mcu::Stm {
    inline namespace V3{}
}

#include "devices.h"

#include <chrono>
#include <cassert>
#include <numbers>

#include <etl/circularbuffer.h>

#include "../include/config.h"
#include "../include/cmsis.h"
#include "../include/rlestimator.h"
#include "../include/identifier.h"
#include "../include/gnuplot.h"
#include "../include/plotadapter.h"
#include "../include/fftestimator.h"
#include "../include/subsampler.h"
#include "../include/eeprom.h"
#include "../include/crsf_cb.h"
#include "../include/fsmRun.h"
#include "../include/fsmRL.h"
#include "../include/fsmKM.h"
#include "../include/toneplay.h"
#include "../include/pid.h"

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

    using pwm = devs::pwm;

    using adc = devs::adc;
    using adcDmaChannel = devs::adcDmaChannel;
#ifndef TEST_C1
    using pga = devs::pga;
#endif
    using offset = devs::offset;

    using servo = devs::servo;
    using servo_pa = devs::servo_pa;
    using crsf_out = devs::crsf_out;
    using sensor_uart = devs::sensor_uart;
    using crsfCallback= devs::crsfCallback;
    using crsfTelemetry = crsfCallback::crsfTelemetry;

    static inline constexpr uint16_t fSize = 1024;
    // static inline constexpr uint16_t fSize = 2048; // RAM overflow
    using subSampler = Dsp::SubSampler<devs, fSize>;
    using estimator = Dsp::FFTEstimator<fSize, subSampler>;

    using identifier = BdcIdentifier<devs>;
    using config = Config<estimator, subSampler, pwm>;

    using rlfsm = RLFsm<systemTimer, pwm, identifier, config, trace>;
    // using rlfsm = RLFsm<systemTimer, pwm, identifier, config, void>;

    using kmfsm = KmFsm<systemTimer, pwm, estimator, config, trace>;
    // using kmfsm = KmFsm<systemTimer, pwm, estimator, config, void>;

    using toneplay = External::TonePlayer<pwm, systemTimer, Storage, trace>;
    // using toneplay = External::TonePlayer<pwm, systemTimer, Storage, void>;

#ifdef USE_GNUPLOT
    // using plot = Graphics::Gnuplot<trace, Meta::List<Adapter0<estimator>, Adapter1<estimator>, Adapter3<estimator>, Adapter4<estimator>, Adapter5<estimator>>>;
    // using plot = Graphics::Gnuplot<trace, Meta::List<Adapter0<estimator>, Adapter1<estimator>, Adapter2<estimator>, Adapter3<estimator>, Adapter4<estimator>, Adapter5<estimator>>>;
    using plot = Graphics::Gnuplot<trace, Meta::List<Adapter1<estimator>, Adapter2<estimator>, Adapter3<estimator>, Adapter4<estimator>, Adapter5<estimator>>>;
    // using plot = Graphics::Gnuplot<trace, Meta::List<Adapter0<estimator>, Adapter1<estimator>, Adapter3<estimator>, Adapter4<estimator>>>;
    // using plot = Graphics::Gnuplot<hsout, Meta::List<Adapter0<estimator>>>;
#endif

    static inline void init() {
        devs::init();
        config::init();
        estimator::init();

        servo_pa::Responder::address(std::byte{0xca}); // ESC address (needs PR wmaddress for ELRS)
    }

    enum class State : uint8_t {Undefined = 0x00,
                                Init,
                                Check, Run,
                                PlayStartupTone,
                                PlayRunTone,
                                PlayWaitTone,
                                PlayTone,
                                PrintMeasures1,
                                PrintMeasures2,
                                StartCalibrate,
                                StopCalibrate,
                                UpdateRL,
                                MeasRL,
                                MeasRot
                               };
    enum class Event : uint8_t {None,
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
    static inline void updateWindow() {
        estimator::windowFunction(Storage::eeprom.timeDomainWindow);
    }

    static inline void updatePwm() {
        const uint16_t cutoff = Storage::eeprom.cutoff_freq * 100;
        config::pwm(Storage::eeprom.subsampling * cutoff * Storage::eeprom.n_fsample);
        config::cutoff(cutoff);
        config::subSampleFactor(Storage::eeprom.subsampling);
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
        case State::StartCalibrate:
            break;
        case State::StopCalibrate:
            break;
        case State::PrintMeasures1:
            break;
        case State::PrintMeasures2:
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

        // comp1::ratePeriodic();

        ++mStateTick;
        ++mTelemTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::PlayStartupTone;
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
                if (input.absolute() <= Storage::eeprom.prerun_hyst) {
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
                mState = State::PrintMeasures1;
            }
            break;
        case State::PrintMeasures1:
            mState = State::PrintMeasures2;
            break;
        case State::PrintMeasures2:
            mState = State::StopCalibrate;
            break;
        case State::Check:
        {
            const auto input = servo_pa::normalized(Storage::eeprom.crsf_channel - 1);
            mStateTick.on(initTicks, [&]{
                IO::outl<trace>("# ch: ", Storage::eeprom.crsf_channel, " v: ", (int16_t)input, " conn: ", (uint8_t)mLinkStatus);
            });
            if (mLinkStatus == LinkStatus::Connected) {
                if (input.absolute() <= Storage::eeprom.prerun_hyst) {
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
                const auto inputFiltered = Speed::dutyFilter.process(input);

                estimator::dir1(inputFiltered >= 0);
                estimator::update(inputFiltered);
                const uint16_t rpm = estimator::eRpm() / Storage::eeprom.telemetry_polepairs;

                if (inputFiltered.absolute() < 10) {
                    if (subSampler::currMean() < 20) {
                        crsfTelemetry::rpm1(0);
                    }
                    else {
                        if (rpm >= 26) {
                            crsfTelemetry::rpm1(rpm);
                        }
                        else {
                            crsfTelemetry::rpm1(0);
                        }
                    }
                }
                else {
                    if (rpm >= 26) {
                        crsfTelemetry::rpm1(rpm);
                    }
                    else {
                        crsfTelemetry::rpm1(0);
                    }
                }


                crsfTelemetry::batt(devs::adc2Voltage(subSampler::meanVoltage()) * 10);

                if (Storage::eeprom.current_select == 0) {
                    crsfTelemetry::curr(devs::adc2Current((subSampler::currMean() - 10) * inputFiltered.absolute() / inputFiltered.absolute().Upper) * 10);
                }
                else {
                    crsfTelemetry::curr(devs::adc2Current((subSampler::currMean() - 10)) * 10);
                }

                // const uint16_t t = Speed::tempFilter.process(Mcu::Stm::adc2Temp(adc::mData[2]));
                const uint16_t t = Speed::tempFilter.process(Mcu::Stm::adc2Temp(adc::values()[2]));
                crsfTelemetry::temp1(t);

                // const uint16_t t2 = comp1::temperatur();
                // crsfTelemetry::temp2(t2);

                // const float setRpm = (vf.absolute() / 1000.0f) * 5000;
                // const float co = mPid.process(setRpm, rpm);
                // const float cc = 1640.0f * (setRpm + co) / 5000.0f;
                // float s = std::max(std::min(cc, 1640.0f), 0.0f);

                if (const auto b = inputFiltered.absolute(); inputFiltered >= 0) {
                    // decltype(b)::_;
#ifdef TEST_C1
#else
                    offset::set();
#endif
                    subSampler::invert(true);
                    pwm::dir1();
                    const float trigger = mTriggerMin + (mTriggerMax - mTriggerMin) * ((float)b.Upper - b.toInt()) / ((float)b.Upper - b.Lower);
                    pwm::trigger(trigger);
                    pwm::duty(b);
                }
                else {
#ifdef TEST_C1
#else
                    offset::reset();
#endif
                    subSampler::invert(false);
                    pwm::dir2();
                    const float trigger = mTriggerMin + (mTriggerMax - mTriggerMin) * ((float)b.Upper - b.toInt()) / ((float)b.Upper - b.Lower);
                    pwm::trigger(trigger);
                    pwm::duty(b);
                }

                // Test code for estimating Rm
                const float km1 = Storage::eeprom.eKm.dir1;
                const float ubatt = estimator::uBattMean();
                const float d = inputFiltered.absolute();
                im = (estimator::currMean() * d) / 1000;
                ue = ubatt * d / 1000;
                um = ((float)rpm) / km1;
                lastRm = (ue - um) / im;
            });
            mStateTick.on(initTicks, [&]{
                const auto input = servo_pa::normalized(Storage::eeprom.crsf_channel - 1);
                const uint16_t t = Speed::tempFilter.value();
                // IO::outl<trace>("# input: ", input.toInt(), " Mcu Temp: ", t, " raw: ", adc::mData[2], " traw: ", (uint16_t)(10 * Mcu::Stm::adc2Temp(adc::mData[2])), " TCAL1: ", *TEMPSENSOR_CAL1_ADDR, " TCAL2: ", *TEMPSENSOR_CAL2_ADDR);
                IO::outl<trace>("# input: ", input.toInt(), " Mcu Temp: ", t, " raw: ", adc::values()[2], " traw: ", (uint16_t)(10 * Mcu::Stm::adc2Temp(adc::values()[2])), " TCAL1: ", *TEMPSENSOR_CAL1_ADDR, " TCAL2: ", *TEMPSENSOR_CAL2_ADDR);
                IO::outl<trace>("# meanADC: ", (uint16_t)subSampler::currMeanADC(), " mean: ", (uint16_t)subSampler::currMean(), " g:", subSampler::gain(), " volt: ", (uint16_t)(10.0f * devs::adc2Voltage(subSampler::meanVoltage())));
            });
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
                IO::outl<trace>("# Play Startup Tone");
                toneplay::event(toneplay::Event::PlayTune1);
                break;
            case State::PlayRunTone:
                IO::outl<trace>("# Play Run Tone");
                toneplay::event(toneplay::Event::PlayTune3);
                break;
            case State::PlayWaitTone:
                IO::outl<trace>("# Play Wait Tone");
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
            case State::StopCalibrate: {
                IO::outl<trace>("# Stop Cal");
                const auto Rm = rlfsm::getLastRm();
                const auto Lm = rlfsm::getLastLm();
                const auto eKm = kmfsm::getMeanEKm();
                Storage::eeprom.resistance = Rm;
                Storage::eeprom.inductance = Lm;
                Storage::eeprom.eKm = eKm;
                crsfCallback::update();
                crsfCallback::save();
                IO::outl<trace>("# Setting Rm: ", (uint16_t)(1000 * Rm.dir1), " Lm: ",  (uint16_t)(1000 * Lm.dir1), " eKm d1: ", (uint16_t)eKm.dir1, " eKm d2: ", (uint16_t)eKm.dir2);
                }
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
            case State::PrintMeasures1:
                IO::outl<trace>("# PrintMeasures1");
                rlfsm::template print<trace>();
                break;
            case State::PrintMeasures2:
                IO::outl<trace>("# PrintMeasures2");
                kmfsm::template print<trace>();
                break;
            case State::Check:
                IO::outl<trace>("# Check");
                break;
            case State::Run:
                IO::outl<trace>("# Run");
                led::event(led::Event::Slow);
                IO::outl<trace>("# ekm d1: ", Storage::eeprom.eKm.dir1, " ekm d2: ", Storage::eeprom.eKm.dir2);
                config::init();
                pwm::duty(0);
                pwm::setSingleMode();
                subSampler::reset();
                break;
            }
        }
    }
    static inline bool isCalibrating() {
        return mState != State::Run;
    }
    static inline uint8_t getCalibrateStateNumber() {
        switch(mState) {
        case State::StartCalibrate:
            return 1;
            break;
        case State::MeasRL:
            return 2;
            break;
        case State::MeasRot:
            return 3 + kmfsm::isDir1();
            break;
        case State::PrintMeasures1:
        case State::PrintMeasures2:
        case State::StopCalibrate:
            return 5;
            break;
        default:
            return 0;
            break;
        }
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
        if (adc::mcuAdc->ISR & ADC_ISR_EOC) {
            adc::mcuAdc->ISR = ADC_ISR_EOC;
        }
        if (adc::mcuAdc->ISR & ADC_ISR_EOS) {
            adc::mcuAdc->ISR = ADC_ISR_EOS;
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
        if (pwm::pwmTimer->SR & TIM_SR_CC1IF) {
            pwm::pwmTimer->SR = ~TIM_SR_CC1IF;
            devs::tp1::set();
            if (inRLMeasuringState()) {
                if (const auto rl = identifier::calculateIsr()) {
                    // IO::outl<trace>("# Rm: ", (uint16_t)(1000 * rl->Rm), " Lm: ", (uint16_t)(rl->Lm));
                    rlfsm::addMeasurement(*rl, dir);
                }
                if (dir) {
#ifndef TEST_C1
                    offset::set();
                    pga::useOffset(true);
#endif
                    identifier::invert(true);
                    pwm::dir1();
                }
                else {
#ifndef TEST_C1
                    offset::reset();
                    pga::useOffset(false);
#endif
                    identifier::invert(false);
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
        if (pwm::pwmTimer->SR & TIM_SR_UIF) {
            pwm::pwmTimer->SR = ~TIM_SR_UIF;
            if (inRLMeasuringState()) {
                devs::tp1::set();
                identifier::startMeasure();
                devs::tp1::reset();
            }
        }
    }
    private:
    static inline const float mTriggerMin = 0.5f;
    static inline const float mTriggerMax = 0.9f;
    static inline PID<float> mPid{1000.0f, -1000.0f, 0.2f, 0.1f, 0.005f};
    static inline float im;
    static inline float ue;
    static inline float um;
    static inline float lastRm;
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
    static inline uint8_t getCalibrateStateNumber() {
        return gfsm::getCalibrateStateNumber();
    }
    static inline void updatePwm() {
        gfsm::updatePwm();
    }
    static inline void resetParameter() {
        gfsm::resetParameter();
    }
    static inline void updateWindow() {
        gfsm::updateWindow();
    }
};

int main() {
    Storage::init();
    gfsm::init();
    gfsm::update();

    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_SetPriority(ADC1_2_IRQn, 1);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 1);

    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, 1);

    // NVIC_EnableIRQ(COMP1_2_3_IRQn);
    // NVIC_SetPriority(COMP1_2_3_IRQn, 0);

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

// void COMP1_2_3_IRQHandler() {
//     devs::tp2::set();
//     if (EXTI->PR1 & EXTI_PR1_PIF21) {
//         EXTI->PR1 = EXTI_PR1_PIF21;
//         // devs::comp1::isr();
//     }
//     devs::tp2::reset();
// }

void ADC1_2_IRQHandler() {
    gfsm::adcIsr();
}

void TIM3_IRQHandler() {
    gfsm::pwmIsr();
}

void TIM4_IRQHandler() {
    devs::tp2::set();
    if (gfsm::pwm::adcTimer->SR & TIM_SR_CC1IF) {
        gfsm::pwm::adcTimer->SR = ~TIM_SR_CC1IF;
    }
    if (gfsm::pwm::adcTimer->SR & TIM_SR_UIF) {
        gfsm::pwm::adcTimer->SR = ~TIM_SR_UIF;
    }
    devs::tp2::reset();
}

}
