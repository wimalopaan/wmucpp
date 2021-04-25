#define NDEBUG

#define LEARN_DOWN

#ifndef GITMAJOR
# define VERSION_NUMBER 2300
#endif

#ifndef NDEBUG
static unsigned int assertKey{1234};
#endif

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <mcu/internals/syscfg.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/rc/busscan.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

struct Data final : public EEProm::DataBase<Data> {
    uint8_t mMagic{};

    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    search_t mChannel{};
    
    uint16_t mThrMin{0};
    uint16_t mThrMax{65535};
    uint16_t mThrMid{32000};
//    uint16_t mThrHalf{16000};
    uint16_t mThrDead{16000};
    
};

template<typename BusDevs>
struct GlobalFsm {
    using gfsm = GlobalFsm<BusDevs>;
    
    using bus_type = BusDevs::bus_type;
    using devs = BusDevs::devs;
    using Led = devs::led;
    using Timer = devs::systemTimer;
    using NVM = devs::eeprom;
    using data_t = devs::eeprom::data_t;
    using PWM = devs::pwm;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;

    using Servo = BusDevs::servo;
    using servo_pa = BusDevs::servo_pa;

    using blinkLed = External::Blinker2<Led, Timer, 100_ms, 2500_ms>;
    using count_type = blinkLed::count_type;

    using lut0 = devs::lut0;
    using lut1 = devs::lut1;
    
    static inline auto& data = NVM::data();

    enum class State : uint8_t {Undefined, Init, NoSignal, Off, forward, Backward};
    
    static constexpr External::Tick<Timer> stateChangeTicks{500_ms};
    static constexpr External::Tick<Timer> resetTicks{1000_ms};

    using throttle_type = typename servo_pa::value_type;
//    throttle_type::_;
    using thr_t = typename throttle_type::value_type;
    
    static inline thr_t thrMax() {
        return NVM::data().mThrMax;
    }
    static inline thr_t thrMin() {
        return NVM::data().mThrMin;
    }
    static inline thr_t thrDead() {
        return NVM::data().mThrDead;
    }
    static inline thr_t thrMedium() {
        return NVM::data().mThrMid;
    }
    
    static inline thr_t thrStartForward() {
        return thrMedium() + thrDead();
    };
    static inline thr_t thrStartBackward() {
        return thrMedium() - thrDead();
    };
    
    inline static bool isThrottleForward() {
        return mThrottle && (mThrottle.toInt() >= thrStartForward());
    }
    inline static bool isThrottleBackward() {
        return mThrottle && (mThrottle.toInt() <= thrStartBackward());
    }
    inline static bool isThrottleOn() {
        return isThrottleBackward() || isThrottleForward();
    }
    
    template<bool full = false>
    inline static void nvmDefaults() {
        NVM::data().mThrMid = (throttle_type::Upper + throttle_type::Lower) / 2;
        NVM::data().mThrMin = throttle_type::Lower;
        NVM::data().mThrMax = throttle_type::Upper;       
        NVM::data().mThrDead= (throttle_type::Upper - throttle_type::Lower) / (2 * 10);
//        if constexpr(full) {
//            searchCh().setToBottom();
//        }
//        data.param(Param::SPortID) = 3;
//        data.param(Param::NSensors) = maxParamValue;
        
        data.change();
    } 
    
    static inline void init(const bool) {
        lut0::init(0xff_B); // in1 = H 
        lut1::init(0xff_B); // in2 = H 
        NVM::init();
        if (data.mMagic != BusDevs::magic) {
            data.mMagic = BusDevs::magic;
            nvmDefaults<true>();
            etl::outl<terminal>("e init"_pgm);
        }
        if constexpr(External::Bus::isPpm<bus_type>::value) {
            if constexpr(!std::is_same_v<TermDev, void>) {
                TermDev::template init<BaudRate<115200>>();
                TermDev::template rxEnable<false>();
            }
            etl::outl<terminal>("PPm"_pgm);
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
        blinkLed::init();
        PWM::init();
//        PWM::period(mPwmPeriodMin());
        PWM::template off<Meta::List<AVR::PWM::WO<0>>>();
    }
    
    static inline void periodic() {
        NVM::saveIfNeeded([&]{
            etl::outl<terminal>("save eep"_pgm);
        });
        Servo::periodic();
        if constexpr(External::Bus::isPpm<bus_type>::value) {
            if constexpr(!std::is_same_v<TermDev, void>) {
                TermDev::periodic();
            }
        }
    }

    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        blinkLed::ratePeriodic();

        mThrottle = servo_pa::value(0);
        
        const auto oldState = mState;
        ++mStateTick;
        (++mResetTick).on(resetTicks, []{
            if (servo_pa::packages() == 0) {
                mState = State::NoSignal;
            }
            servo_pa::resetStats();
            data.expire();
        });
        switch(mState) {
        case State::Undefined:
            mStateTick.on(stateChangeTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if (servo_pa::packages() > 10) {
                mState = State::Off;
            }
            break;            
        case State::NoSignal:
            if (servo_pa::packages() > 10) {
                mState = State::Off;
            }
            break;            
        case State::Off:
            if (isThrottleForward()) {
                mState = State::forward;
            }
            else if (isThrottleBackward()) {
                mState = State::Backward;
            }
            break;            
        case State::forward:
            if (!isThrottleForward()) {
                mState = State::Off;
            }
            else {
                const auto tv = mThrottle.toInt();
                const auto d = etl::distance(tv, thrStartForward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax() - thrStartForward()}, etl::Intervall{0u, PWM::max()});
                PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(pv);
                
            }
            break;            
        case State::Backward:
            if (!isThrottleBackward()) {
                mState = State::Off;
            }
            else {
                
            }
            break;            
        }
        if (mState != oldState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                blinkLed::off();
                break;
            case State::Init:
                blinkLed::steady();
                break;            
            case State::NoSignal:
                blinkLed::blink(count_type{4});
                break;            
            case State::Off:
                blinkLed::blink(count_type{1});
                break;            
            case State::forward:
                blinkLed::blink(count_type{2});
                lut0::init(0x01_B); // in1 = !WO0
                lut1::init(0x00_B); // in2 = 0
                break;            
            case State::Backward:
                blinkLed::blink(count_type{3});
                lut0::init(0x01_B); // in1 = !WO0
                lut1::init(0x02_B); // in2 = WO0
                break;            
            }
        }
    }
private:
    static inline throttle_type mThrottle;
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mResetTick;
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;

    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>; 
        
    using servoPosition = usart0Position; 
    using scanDevPosition = servoPosition;
    
    using sensorPosition = void; // Sensor
    using scanTermPosition = sensorPosition;
#ifdef NDEBUG
    using scan_term_dev = void;
#else
    using scan_term_dev = Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#endif
    
    using ledPin = Pin<Port<A>, 1>; 
    using scanLedPin = ActiveHigh<ledPin, Output>;
    using led = scanLedPin;
#ifndef NDEBUG
    using assertPin = ledPin; 
#endif
    
    using ccl0Position = Portmux::Position<Component::Ccl<0>, Portmux::Default>;
    using lut0 = Ccl::SimpleLut<0, Ccl::Input::Tca0<0>, Ccl::Input::Mask, Ccl::Input::Mask>;
    using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;
    using lut1 = Ccl::SimpleLut<1, Ccl::Input::Tca0<0>, Ccl::Input::Mask, Ccl::Input::Mask>;
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Alt1>;
    
    using pwm = PWM::DynamicPwm<tcaPosition>;
    
    using sppmPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
    using ppmIn =  AVR::Pin<AVR::Port<A>, 2>;
    using evch0 = Event::Channel<0, Event::Generators::Pin<ppmIn>>; 
    using ppm_user = Event::Route<evch0, Event::Users::Tcb<0>>;
    using ppmDevPosition = AVR::Component::Tcb<0>;

    using allRoutes = Event::Routes<ppm_user>;
    using evrouter = Event::Router<Event::Channels<evch0>, allRoutes>;
    
    using eeprom = EEProm::Controller<Data>;
    
    using portmux = Portmux::StaticMapper<Meta::List<ccl0Position, ccl1Position, tcaPosition, servoPosition, sppmPosition>>;
    
    static inline void init() {
        portmux::init();
        evrouter::init();
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDa32<MCU>) {
//                clock::template init<Project::Config::fMcuMhz>();
            }
            else if constexpr(AVR::Concepts::At01Series<MCU>) {
                clock::template prescale<1>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
    }
    static inline void periodic() {
    }
};


template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::Ppm<Devs>> {
    static inline uint8_t magic = 45;
    using bus_type = External::Bus::Ppm<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using sppm_input = External::Ppm::SinglePpmIn<typename devs::sppmPosition::component_type>;
     
    using servo = External::Ppm::Adapter<sppm_input>;
    using servo_pa = servo::protocoll_adapter_type;
    
#ifndef NDEBUG
    using term_dev = Usart<typename devs::servoPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
    using terminal = etl::basic_ostream<term_dev>;
#else
    using terminal = etl::basic_ostream<void>;
#endif

};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isPpm<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GlobalFsm<devs>;
            
            gfsm::init(inverted);

            etl::outl<terminal>("esc mini_10_hw21"_pgm);
            
            while(true) {
                gfsm::periodic(); 
                systemTimer::periodic([&]{
                    gfsm::ratePeriodic();
                });
            }
        }
    }
};

using devices = Devices<>;
using scanner = External::Scanner2<devices, Application, Meta::List<External::Bus::Ppm<devices>>>;

static_assert(scanner::checkPpm);
static_assert(!scanner::checkSbus);

int main() {
    scanner::run();
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    xassert::ab.clear();
    xassert::ab.insertAt(0, expr);
    etl::itoa(line, xassert::aline);
    xassert::ab.insertAt(20, xassert::aline);
    xassert::ab.insertAt(30, file);
    xassert::on = true;
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
