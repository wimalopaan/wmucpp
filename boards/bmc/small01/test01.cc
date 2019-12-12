#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/event.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/ccl.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
//#include <external/hott/sumdprotocolladapter.h>
//#include <external/hott/experimental/sensor.h>
//#include <external/hott/hott.h>
//#include <external/hott/menu.h>

#include <external/solutions/series01/sppm_in.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

struct Storage final {
    enum class AVKey : uint8_t {MaxPpm = 0, MinPpm, Init1, Init2, _Number};
    
    struct ApplData : public EEProm::DataBase<ApplData> {
        uint16_t& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<uint16_t, static_cast<uint8_t>(AVKey::_Number)> AValues{};
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using PortA = Port<A>;
using eflag = Pin<PortA, 2>; 

using ppmIn =  AVR::Pin<PortA, 3>;

using ppmTimerPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using tcdPosition = Portmux::Position<Component::Tcd<0>, Portmux::Default>;
using pwm = PWM::DynamicPwm<tcdPosition>;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
    constexpr auto fRtc = 512_Hz;

    constexpr float cell_normal = 2.0f;
    constexpr float cell_min = 1.7f;
    
    constexpr float voltage_divider = 0.1488f;
}

using systemTimer = SystemTimer<Component::Pit<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 2>;

#ifdef USE_HOTT
#else
using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0>>;

//using rtc_channel = Event::Channel<0, Event::Generators::PitDiv<1024>>;
using ppm_channel = Event::Channel<0, Event::Generators::Pin<ppmIn>>; 
//using ac_channel = Event::Channel<2, Event::Generators::Ac0<Event::Generators::Kind::Out>>; 
//using userstrobe_channel = Event::Channel<3, void>; 
using ppm_user = Event::Route<ppm_channel, Event::Users::Tcb<0>>;
//using ac_user = Event::Route<ac_channel, Event::Users::Tcb<2>>;
//using userstrobe_user = Event::Route<userstrobe_channel, Event::Users::Tcb<2>>;
using evrouter = Event::Router<Event::Channels<ppm_channel>, Event::Routes<ppm_user>>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcdPosition, ppmTimerPosition>>;

struct TLE {
    
};

using bridge = TLE;

template<typename Actuator, uint8_t MaxIncrement = 10> 
struct Controller {
    static inline void init() {
        Actuator::init();
    }
    template<typename ValueType>
    static inline void set(const ValueType& v, bool b) {
        using signed_type = std::make_signed_t<typename ValueType::value_type>;
        signed_type diff = std::min<signed_type>(mThresh, v.toInt() - lastValue);
        
        ValueType nv = lastValue + diff;
        Actuator::set(nv, b);
        lastValue = nv.toInt();
    }
    static inline void off() {
        Actuator::off();
        lastValue = 0;
    }
private:
    static inline uint16_t lastValue;
    static inline const uint16_t mThresh = MaxIncrement;
};

using controller = Controller<bridge>;

template<typename Actuator>
struct EscStateFsm {
    enum class State : uint8_t {Undefined = 0, Backward, Off, Forward, _Number};
    
    static inline void init() {
        Actuator::init();
    }    
    template<typename InputType>
    static inline void update(const InputType& v) {
        using output_t = etl::uint_ranged_NaN<std::make_unsigned_t<typename InputType::value_type>, 0, InputType::Upper>;
        
        if (!v) return;
        
        switch(mState) {
        case State::Undefined:
            mState = State::Off;
            break;
        case State::Backward:
            if (v.toInt() >= 0) {
                mState = State::Off;
                mOffStateCounter = 0;
            }
            else {
                Actuator::set(output_t(-v.toInt()), true);
            }
            break;
        case State::Off:
            Actuator::off();
            if (++mOffStateCounter > 5) {
                if (v.toInt() > 0) {
                    mState = State::Forward;
                }
                if (v.toInt() < 0) {
                    mState = State::Backward;
                }
            }
            break;
        case State::Forward:
            if (v.toInt() <= 0) {
                mState = State::Off;
                mOffStateCounter = 0;
            }
            else {
                Actuator::set(output_t(v.toInt()), false);
            }
            break;
        default:
            break;
        }
    }
private:
    static inline State mState = State::Undefined;
    static inline etl::uint_ranged_NaN<uint8_t, 0, 100> mOffStateCounter = 0;
};

using fsm = EscStateFsm<controller>;

template<typename TUnit, typename AD>
struct GlobalFSM {
    enum class State : uint8_t {Undefined = 0, Startup, CellEstimate, CellBeep, ThrottleWarn, ThrottleSetMax, ThrottleSetMin, ThrottleSetNeutral, ThrottleSetCheck, 
                                Error, Armed, Run, RunReduced, _Number};
    enum class Beep  : uint8_t {Off, Low, High, _Number};
    
    static inline constexpr auto intervall = TUnit::exact_intervall;
//    std::integral_constant<uint16_t, intervall.value>::_;
    static inline constexpr auto minStateTime = 500_ms;
    static inline constexpr auto maxStateTime = 3000_ms;
    using state_counter_type = etl::typeForValue_t<(maxStateTime / intervall) + 1>;
    static inline constexpr state_counter_type minStateCounter = minStateTime / intervall;
    static inline constexpr state_counter_type maxStateCounter = maxStateTime / intervall;
    
    using ppm_type = decltype(ppm::value());
    
    static inline constexpr auto ppmHysterese = 2 * (ppm_type::Upper - ppm_type::Lower) / 100;
    
    static inline constexpr typename AD::value_type cell_normal_raw = (Parameter::cell_normal * Parameter::voltage_divider * 1024) / AD::VRef;
//    std::integral_constant<uint16_t, cell_normal_raw.toInt()>::_;
    static inline constexpr typename AD::value_type cell_min_raw = (Parameter::cell_min * Parameter::voltage_divider * 1024) / AD::VRef;
//    std::integral_constant<uint16_t, cell_min_raw.toInt()>::_;
    
    static inline void init() {
        mMaxPpm = appData[Storage::AVKey::MaxPpm];
        mMinPpm = appData[Storage::AVKey::MinPpm];
    }

    static inline void saveToEEprom() {
        appData[Storage::AVKey::MaxPpm] = mMaxPpm;
        appData[Storage::AVKey::MinPpm] = mMinPpm;
        appData.change();
    }
    
    template<typename V>
    static inline bool isNearNeutral(const V& v) {
        return ((v >= -ppmHysterese) && (v <= ppmHysterese));
    }
    
    template<typename T>
    static inline ppm_type scale(const T& v) {
        using tt = etl::enclosing_t<ppm_type::value_type>;
        tt scaled = ((tt)v.toInt() * ppm_type::Upper) / mMaxPpm;
        scaled = std::clamp(scaled, (tt)ppm_type::Lower, (tt)ppm_type::Upper);
        return scaled;
    }
    
    static inline void periodic() { // called every 'intervall' ms
        auto p = ppm::value();
//        decltype(p)::_;
        State oldState = mState;
        switch (mState) {
        case State::Undefined:
            if (p && (++mStateCounter > minStateCounter)) {
                mState = State::Startup;
            }
            break;
        case State::Startup:
            if (p && (++mStateCounter > minStateCounter)) {
                if (!isNearNeutral(p)) {
                    mState = State::ThrottleWarn;
                }
                else {
                    mState = State::CellEstimate;
                }
            }
            break;
        case State::CellEstimate:
        {
//            auto v = AD::value(1);
//            mCellCount = v.toInt() / cell_normal_raw;
//            mVThresh = mCellCount * cell_min_raw.toInt();
//            mState = State::CellBeep;
        }
            break;
        case State::CellBeep:
            if (++mStateCounter > maxStateCounter) {
                mState = State::Armed;
            }
            break;
        case State::ThrottleWarn:
            if (p && isNearNeutral(p)) {
                mState = State::Run;
            }
            break;
        case State::ThrottleSetMax:
            if (p) {
                mMaxPpm = p.toInt();
            }
            break;
        case State::ThrottleSetNeutral:
            break;
        case State::ThrottleSetMin:
            if (p) {
                mMinPpm = p.toInt();
            }
            break;
        case State::ThrottleSetCheck:
//            if ((mMaxPpm > 300) && (mMinPpm < -300)) {
//                saveToEEprom();
//                if (++mStateCounter > minStateCounter) {
//                    mState = State::Startup;
//                }
//            }
//            else {
//                mState = State::Error;
//            }
            break;
        case State::Error:
            break;
        case State::Armed:
            if (++mStateCounter > minStateCounter) {
                mState = State::Run;
            }
            break;
        case State::Run:
        {
//            auto v = AD::value(1);
//            if (v < mVThresh) {
//                mState = State::RunReduced;
//            }               
//            if (p) {
//                mScaled = scale(p);
////                fsm::update(mScaled);
//            }
        }
            break;
        case State::RunReduced:
        {
            if (p) {
                mScaled = scale(p);
                mScaled /= 2;
//                fsm::update(mScaled);
            }
        }
            break;
        default:
            break;
        }
        if (oldState != mState) {
            mStateCounter = 0;
            switch (mState) {
            case State::Undefined:
                break;
            case State::Startup:
                break;
            case State::CellEstimate:
                break;
            case State::CellBeep:
                break;
            case State::ThrottleWarn:
                break;
            case State::ThrottleSetMax:
                mMaxPpm = 300;
                mMinPpm = -300;
                break;
            case State::ThrottleSetNeutral:
                break;
            case State::ThrottleSetMin:
                break;
            case State::ThrottleSetCheck:
                break;
            case State::Error:
                break;
            case State::Armed:
                break;
            case State::Run:
                break;
            case State::RunReduced:
                break;
            default:
                break;
            }
        }
    }
    
    static inline State mState = State::Undefined;
    static inline ppm_type mScaled;
    static inline ppm_type::value_type mMaxPpm = ppm_type::Upper;
    static inline ppm_type::value_type mMinPpm = ppm_type::Lower;
    static inline AD::value_type mVThresh = 0;
    static inline uint8_t mCellCount = 0;
private:
    static inline etl::uint_ranged<state_counter_type, 0, maxStateCounter + 1> mStateCounter;
};

using gfsm = GlobalFSM<systemTimer, adcController>;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    eflag::template dir<Input>();
    eflag::template pullup<true>();
    
    evrouter::init();
    portmux::init();
    eeprom::init();
    systemTimer::init();
    
    ppm::init();
    
    terminalDevice::init<AVR::BaudRate<9600>>();
    
    gfsm::init();

    
    pwm::init();
    pwm::frequency(300_Hz);
//    pwm::template duty<PWM::WO<0>>(pwm::max() / 5);
//    pwm::template on<Meta::List<PWM::WO<0>>>();
//    pwm::template off<Meta::List<PWM::WO<1>>>();
    pwm::template clear<PWM::WO<0>>(0);
    pwm::template set<PWM::WO<0>>(pwm::max() / 10);
    pwm::template set<PWM::WO<1>>(pwm::max() / 10);
    

    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    uint8_t counter = 0;

#ifndef USE_HOTT
    etl::outl<terminal>("test01"_pgm);
#endif
    
    bool eepSave = false;
    while(true) {
        eepSave |= eeprom::saveIfNeeded();
#ifdef USE_HOTT
#else
        terminalDevice::periodic();
#endif
        systemTimer::periodic([&]{
            gfsm::periodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    ++counter;
                    if ((counter % 2) == 0) {
#ifndef USE_HOTT
                        etl::outl<terminal>("ppm: "_pgm, ppm::value().toInt());
                        etl::outl<terminal>("pwm max: "_pgm, pwm::max().toInt());
#endif
                    }
                    else {
                    }
                }
            });
            appData.expire();
        });
    }
}

