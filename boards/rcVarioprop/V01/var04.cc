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

    using btUsart = devs::btUsart;
    using btPa = devs::btPa;

    using adc1 = devs::adc1;
    using adc2 = devs::adc2;

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

        btUsart::periodic();
    }

    enum class State : uint8_t {Undefined, Init, Run, Reset,
                                Set};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> changeTicks{2000ms};

    static inline uint8_t txChannel = 56; // C8
    // static inline uint8_t txChannel = 58;
    // static inline uint8_t txChannel = 64; // Scan RX
    // static inline uint8_t txChannel = 65;

    static inline void ratePeriodic() {
        btPa::ratePeriodic();

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
            if (si::setChannel(txChannel)) {
                mState = State::Run;
            }
            break;
        case State::Run:
            (++mDebugTick).on(debugTicks, []{
                IO::outl<trace>("a0: ", adc1::mData[0], " a1: ", adc1::mData[1], " a2: ", adc1::mData[2], " a3: ", adc1::mData[3], " a4: ", adc1::mData[4], " a5: ", adc1::mData[5]);
                IO::outl<trace>("t: ", adc1::mData[6], " v: ", adc2::mData[0]);
                IO::outl<btUsart>("test");

            });
            (++mChangeTick).on(changeTicks, []{
                cppm::setNormalized(4, mToggle ? 100 : 900);
                mToggle = !mToggle;
            });
            for(uint8_t i = 0; i < 4; ++i) {
                const uint16_t v = 999.0f * adc1::mData[i] / 4095.0f;
                cppm::setNormalized(i, v);
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
            case State::Set:
                break;
            case State::Run:
                adc1::start();
                adc2::start();
                // hfPwr::set();
                // led::reset();
                break;
            case State::Reset:
                break;
            }
        }
    }

    static inline uint32_t div{0};
private:
    static inline bool mToggle{false};
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mChangeTick;
};

using devs = Devices<Var01, Config, Mcu::Stm::Stm32G431>;

struct CrsfCallback {
    // using out = crsf_out;
    using out = devs::crsf_out;
    using trace = devs::trace;
    using Param_t = RC::Protokoll::Crsf::Parameter;
    using PType = RC::Protokoll::Crsf::Parameter::Type;

    static inline void setParameter(const uint8_t index, const uint8_t value) {
        IO::outl<trace>("SetP adr: ", index, " v: ", value);
        if ((index >= 1) && (index <= params.size())) {
            params[index - 1].mValue = value;
            switch(index) {
            case 17: // BT
                IO::outl<trace>("BT: ", value);
                if (value) {
                    // btpwr::set(false); // low on pin
                    // btpwr::template dir<Mcu::Output>(); // floating is off
                }
                else {
                    // btpwr::set(true); // high on pin
                    // btpwr::template dir<Mcu::Input>(); // floating
                }
                break;
            default:
                break;
            }
        }
    }
    static inline RC::Protokoll::Crsf::Parameter parameter(const uint8_t index) {
        if ((index >= 1) && (index <= params.size())) {
            return params[index - 1];
        }
        return {};
    }
    static inline bool isCommand(const uint8_t index) {
        if ((index >= 1) && (index <= params.size())) {
            return params[index - 1].mType == PType::Command;
        }
        return false;
    }
    static inline const char* name() {
        return mName;
    }
    static inline uint32_t serialNumber() {
        return mSerialNumber;
    }
    static inline uint32_t hwVersion() {
        return mHWVersion;
    }
    static inline uint32_t swVersion() {
        return mSWVersion;
    }
    static inline uint8_t numberOfParameters() {
        return params.size();
    }
    static inline uint8_t protocolVersion() {
        return 0;
    }
private:
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{1};
    static inline constexpr uint32_t mSWVersion{1};
    static inline constexpr auto mVersionString = [](){
        std::array<char, 16> s{};
        auto [ptr, e] = std::to_chars(std::begin(s), std::end(s), mHWVersion);
        *ptr++ = ':';
        std::to_chars(ptr, std::end(s), mSWVersion);
        return s;
    }();
    static inline constexpr const char* const mName = "CruiseControl";
    static inline etl::FixedVector<Param_t, 64> params {
                                                       Param_t{0, PType::Sel, "P00", "Off;On", 0, 0, 1},
                                                       Param_t{0, PType::Folder, "Serial"}, // 2
                                                       Param_t{2, PType::Sel, "Protocol", "Off;S.Port;SBus;IBus-Sens;IBus-Servo", 0, 0, 4},
                                                       Param_t{2, PType::Sel, "Baudrate", "100k;115k;420k", 0, 0, 2},
                                                       Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]},
                                                       Param_t{0, PType::Folder, "Channels"}, // 6
                                                       Param_t{6, PType::U8, "Output 1", nullptr, 1, 1, 16},
                                                       Param_t{6, PType::U8, "Output 2", nullptr, 2, 1, 16},
                                                       Param_t{6, PType::U8, "Output 3", nullptr, 3, 1, 16},
                                                       Param_t{6, PType::U8, "Output 4", nullptr, 4, 1, 16},
                                                       Param_t{0, PType::Folder, "Modes"},
                                                       Param_t{11, PType::Sel, "Output 1", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
                                                       Param_t{11, PType::Sel, "Output 2", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
                                                       Param_t{11, PType::Sel, "Output 3", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
                                                       Param_t{11, PType::Sel, "Output 4", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
                                                       Param_t{0, PType::Command, "Reset", "Resetting...", 0}, // Command, timeout: 200 = 2s (value 0xc8)
                                                       Param_t{0, PType::Sel, "Blutooth", "Off;On", 0, 0, 1},
                                                       };
};


int main() {
    using gfsm = GFSM<devs>;
    gfsm::init();

    // NVIC_EnableIRQ(ADC1_2_IRQn);
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
    }
}

extern "C" {

void TIM3_IRQHandler() {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;
        devs::pulse::startRollon();
    }
    if (TIM3->SR & TIM_SR_CC1IF) {
        TIM3->SR = ~TIM_SR_CC1IF;
        devs::pulse::startRolloff();
    }
}

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


}
