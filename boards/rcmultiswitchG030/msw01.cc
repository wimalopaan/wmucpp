#define USE_MCU_STM_V3
#define USE_CRSF_V2

#define NDEBUG

#include <cstdint>

#include "devices.h"

using namespace std::literals::chrono_literals;

template<typename Pwm, auto Channel, typename Debug = void>
struct PwmAdapter {
    static inline void duty(const uint8_t d) {
        const uint16_t dv = (Pwm::period * (uint32_t)d) / 255;
        if constexpr(Channel == 0) {
            if constexpr (!std::is_same_v<Debug, void>) {
                IO::outl<Debug>("duty1: ", dv);
            }
            Pwm::duty1(dv);
        }
        else {
            if constexpr(!std::is_same_v<Debug, void>) {
                IO::outl<Debug>("duty2: ", dv);
            }
            Pwm::duty2(dv);
        }
    }
};

template<typename Pin, typename Timer, typename Pwm = void, typename Debug = void>
struct BlinkerWithPwm {
    enum class State : uint8_t {Off, On, IntervallOff, IntervallOn};

    enum class Event : uint8_t {None, Off, On};

    static inline void event(const Event e) {
        mEvent = e;
    }

    static inline void blink(const uint8_t b) {
        if constexpr(!std::is_same_v<Debug, void>) {
            IO::outl<Debug>("Pin: ", Pin::number, " Blink: ", b);
        }
        mBlink = b;
    }

    static inline void on_dezi(const uint8_t b) {
        if constexpr(!std::is_same_v<Debug, void>) {
            IO::outl<Debug>("Pin: ", Pin::number, " Ion: ", b);
        }
        onTicks = External::Tick<Timer>::fromRaw(b * 200);
    }

    static inline void off_dezi(const uint8_t b) {
        if constexpr (!std::is_same_v<Debug, void>) {
            IO::outl<Debug>("Pin: ", Pin::number, " Ioff: ", b);
        }
        offTicks = External::Tick<Timer>::fromRaw(b * 200);
    }

    static inline void pwm(const uint8_t p) {
        if constexpr (!std::is_same_v<Debug, void>) {
            IO::outl<Debug>("Pin: ", Pin::number, " pwm: ", p);
        }
        mUsePwm = p;
    }

    static inline void duty(const uint8_t d) {
        if constexpr(!std::is_same_v<Pwm, void>) {
            if constexpr (!std::is_same_v<Debug, void>) {
                IO::outl<Debug>("Pin: ", Pin::number, " duty: ", d);
            }
            Pwm::duty(d);
        }
    }

    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Off:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::On) {
                if (mBlink) {
                    mState = State::IntervallOn;
                }
                else {
                    mState = State::On;
                }
            }
            break;
        case State::On:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::Off) {
                mState = State::Off;
            }
            break;
        case State::IntervallOff:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::Off) {
                mState = State::Off;
            }
            mStateTick.on(offTicks, []{
                mState = State::IntervallOn;
            });
            break;
        case State::IntervallOn:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::Off) {
                mState = State::Off;
            }
            mStateTick.on(onTicks, []{
                mState = State::IntervallOff;
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Off:
                if constexpr (!std::is_same_v<Debug, void>) {
                    IO::outl<Debug>("Pin: ", Pin::number, " Off");
                }
                Pin::reset();
                Pin::template dir<Mcu::Output>();
                break;
            case State::On:
                if constexpr (!std::is_same_v<Debug, void>) {
                    IO::outl<Debug>("Pin: ", Pin::number, " On");
                }
                if (mUsePwm) {
                    if constexpr (!std::is_same_v<Debug, void>) {
                        IO::outl<Debug>("Pin: ", Pin::number, " Pwm");
                    }
                    Pin::afunction(1); // todo: Mapper bzw. über Pwm
                }
                else {
                    Pin::set();
                    Pin::template dir<Mcu::Output>();
                }
                break;
            case State::IntervallOff:
                if constexpr (!std::is_same_v<Debug, void>) {
                    IO::outl<Debug>("Pin: ", Pin::number, " IntOff");
                }
                Pin::reset();
                Pin::template dir<Mcu::Output>();
                break;
            case State::IntervallOn:
                if constexpr (!std::is_same_v<Debug, void>) {
                    IO::outl<Debug>("Pin: ", Pin::number, " IntOn");
                }
                if (mUsePwm) {
                    Pin::afunction(1);
                }
                else {
                    Pin::set();
                    Pin::template dir<Mcu::Output>();
                }
                break;
            }
        }
    }

    private:
    static inline Event mEvent{Event::None};
    static inline State mState{State::Off};
    static inline bool mUsePwm{false};
    static inline bool mBlink{false};
    static inline External::Tick<Timer> onTicks{500ms};
    static inline External::Tick<Timer> offTicks{500ms};
    static inline External::Tick<Timer> mStateTick;
};

struct McuConfig {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using debug   = Mcu::Stm::Uart<2, void, 256, char, clock>;

    static inline void (*callback)(uint8_t) = nullptr;
};

// template<typename CbList, typename Trace = void>
template<typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using Param_t = RC::Protokoll::Crsf::Parameter;
    using PType = RC::Protokoll::Crsf::Parameter::Type;

    static inline void setParameter(const uint8_t index, const uint8_t value) {
        if constexpr (!std::is_same_v<trace, void>) {
            IO::outl<trace>("SetP adr: ", index, " v: ", value);
        }
        if ((index >= 1) && (index <= params.size())) {
            params[index - 1].mValue = value;
            mLastChangedParameter = index;
            if (params[index - 1].cb) {
                params[index - 1].cb(value);
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
    static inline void whenParameterChanged(auto f) {
        if (mLastChangedParameter > 0) {
            f(mLastChangedParameter);
            mLastChangedParameter = 0;
        }
    }
    template<auto L>
    static inline void command(const std::array<uint8_t, L>& payload) {
        if (payload[0] == (uint8_t)RC::Protokoll::Crsf::Address::Controller) {
            if (payload[2] == (uint8_t)RC::Protokoll::Crsf::CommandType::Switch) {
                if (payload[3] == (uint8_t)RC::Protokoll::Crsf::SwitchCommand::Set) {
                    const uint8_t address = (uint8_t)payload[4];
                    const uint8_t sw = (uint8_t)payload[5];
                    if constexpr (!std::is_same_v<trace, void>) {
                        IO::outl<trace>("Command: ", address, " sw: ", sw);
                    }
                    if (McuConfig::callback) {
                        McuConfig::callback(sw);
                    }
                }
            }
        }
    }
// private:
    static inline uint8_t mLastChangedParameter{};
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
    static inline constexpr const char* const mName = "MultiSwitch-E";
    static inline etl::FixedVector<Param_t, 64> params {
        Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]},
        Param_t{0, PType::U8, "Address", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 0"}, // 3
        // todo: Info als Überschrift
        Param_t{3, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{3, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{3, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{3, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{3, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 1"}, // 9
        Param_t{9, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{9, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{9, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{9, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{9, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 2"}, // 15
        Param_t{15, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{15, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{15, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{15, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{15, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 3"}, // 21
        Param_t{21, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{21, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{21, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{21, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{21, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 4"}, // 27
        Param_t{27, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{27, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{27, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{27, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{27, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 5"}, // 33
        Param_t{33, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{33, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{33, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{33, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{33, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 6"}, // 39
        Param_t{39, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{39, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{39, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{39, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{39, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 7"}, // 45
        Param_t{45, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{45, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{45, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{45, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{45, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},
    };
};

struct Config {
    using mcuconfig = McuConfig;
    using crsfcallback = CrsfCallback<void>;
    // using crsfcallback = CrsfCallback<mcuconfig::debug>;
    using debug = mcuconfig::debug;
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using crsf = devs::crsf;
    using crsf_out = devs::crsf_out;
    using led = devs::led;
    using pb4 = devs::pb4;
    using mco = devs::mco;
    using debug = devs::debug;

    using sw0 = devs::sw0;
    using sw1 = devs::sw1;
    using sw2 = devs::sw2;
    using sw3 = devs::sw3;
    using sw4 = devs::sw4;
    using sw5 = devs::sw5;
    using sw6 = devs::sw6;
    using sw7 = devs::sw7;

    using pwm3 = devs::pwm3;

    using debug1 = void;
    using adap30 = PwmAdapter<pwm3, 0, debug1>;
    using adap31 = PwmAdapter<pwm3, 1, debug1>;

    using bsw0 = BlinkerWithPwm<sw0, systemTimer, void, debug1>;
    using bsw1 = BlinkerWithPwm<sw1, systemTimer, void, debug1>;
    using bsw2 = BlinkerWithPwm<sw2, systemTimer, void, debug1>;
    using bsw3 = BlinkerWithPwm<sw3, systemTimer, void, debug1>;
    using bsw4 = BlinkerWithPwm<sw4, systemTimer, void, debug1>;
    using bsw5 = BlinkerWithPwm<sw5, systemTimer, void, debug1>;
    using bsw6 = BlinkerWithPwm<sw6, systemTimer, adap30, debug1>;
    using bsw7 = BlinkerWithPwm<sw7, systemTimer, adap31, debug1>;

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

    // using crsfcallback = CrsfCallback<bsws, void>;
    using crsfcallback = CrsfCallback<void>;

    enum class State : uint8_t {Undefined, Init, Run};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void set(const uint8_t sw) {
        IO::outl<debug>("set: ", sw);
        if (sw & 0b0000'0001) {
            IO::outl<debug>("set0");
            bsw0::event(bsw0::Event::On);
        }
        else {
            bsw0::event(bsw0::Event::Off);
        }
        if (sw & 0b0000'0010) {
            bsw1::event(bsw1::Event::On);
        }
        else {
            bsw1::event(bsw1::Event::Off);
        }
        if (sw & 0b0000'0100) {
            sw2::set();
        }
        else {
            sw2::reset();
        }
        if (sw & 0b0000'1000) {
            sw3::set();
        }
        else {
            sw3::reset();
        }
        if (sw & 0b0001'0000) {
            sw4::set();
        }
        else {
            sw4::reset();
        }
        if (sw & 0b0010'0000) {
            sw5::set();
        }
        else {
            sw5::reset();
        }
        if (sw & 0b0100'0000) {
            bsw6::event(bsw6::Event::On);
        }
        else {
            bsw6::event(bsw6::Event::Off);
        }
        if (sw & 0b1000'0000) {
            bsw7::event(bsw7::Event::On);
        }
        else {
            bsw7::event(bsw7::Event::Off);
        }
    }

    static inline void init() {
        McuConfig::callback = set;
        crsfcallback::params[3].cb = bsw0::pwm;
        crsfcallback::params[39].cb = bsw6::pwm;
        crsfcallback::params[40].cb = bsw6::duty;
        crsfcallback::params[41].cb = bsw6::blink;
        crsfcallback::params[42].cb = bsw6::on_dezi;
        crsfcallback::params[43].cb = bsw6::off_dezi;
        crsfcallback::params[45].cb = bsw7::pwm;
        crsfcallback::params[46].cb = bsw7::duty;

        devs::init();
    }
    static inline void periodic() {
        crsf::periodic();
        debug::periodic();
        mco::toggle();
    }
    static inline void ratePeriodic() {
        pb4::toggle();
        crsf_out::ratePeriodic();

        bsw0::ratePeriodic();
        bsw1::ratePeriodic();
        bsw2::ratePeriodic();
        bsw3::ratePeriodic();
        bsw4::ratePeriodic();
        bsw5::ratePeriodic();
        bsw6::ratePeriodic();
        bsw7::ratePeriodic();

        ++mStateTick;
        const auto oldState = mState;
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
            (++mDebugTick).on(debugTicks, []{
                led::toggle();
            });
            Config::crsfcallback::whenParameterChanged([](const uint8_t p){

            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                IO::outl<debug>("Init");
                break;
            case State::Run:
                IO::outl<debug>("Run");
                break;
            }
        }
    }
    private:
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState = State::Undefined;
};

using devs = Devices<Nucleo, Config>;
using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    // NVIC_EnableIRQ(TIM3_IRQn);
    // __enable_irq();

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
