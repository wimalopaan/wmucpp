#define NDEBUG

// Nur sbus in/out

// evtl noch lut für TxD-Invertiert

#include <mcu/avr.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/sleep.h>
#include <mcu/internals/dac.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/watchdog.h>

#include <external/solutions/cells.h>
#include <external/solutions/blinker.h>
#include <external/ibus/ibus2.h>
#include <external/sbus/sbus.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/rotaryencoder.h>
#include <external/solutions/series01/sppm_out.h>
#include <external/solutions/series01/cppm_out.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

inline static constexpr auto fRtc = 1000_Hz;
inline static constexpr auto longPress = 1000_ms;

namespace Storage {
    enum class Rotation : uint8_t {Absolute = 1, Bounded, Speed};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        
        uint8_t magic() {
            return mMagic;
        }
        void clear() {
            mMagic = 42;
            mRot = Rotation::Absolute;
            change();
        }
        Rotation& rotation() {
            return mRot;
        }
    private:
        Rotation mRot{Rotation::Absolute};
        uint8_t mMagic{};
    };
}

template<typename HWRev = void>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    using sleep = Sleep<>;
    using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
    
    using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, Portmux::Default>;
    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
    
    using sbus_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using sbus = External::SBus::Output::Generator<usart0Position, systemTimer, void, sbus_pa, 32>;
    
    using outPin = Pin<Port<A>, 1>;
    
    using portmux = Portmux::StaticMapper<Meta::List<tca0Position, usart0Position>>;
    
    using pinA = Pin<Port<A>, 3>;
    using pinB = Pin<Port<A>, 2>;
    
    using rot1_t = uint8_t; // wrap around
    //    using rot1_t = sbus::value_type; // saturating
    using rot2_t = etl::uint_ranged<uint8_t, 0, 255>; // saturating
    //    using rot2_t = etl::cyclic_type_t<rot1_t>; // saturating
    using rotary = External::RotaryEncoder<pinA, pinB, std::variant<rot1_t, rot2_t>>;
    
    using pinT = Pin<Port<A>, 7>;
    using b = ActiveLow<pinT, Input>;
    using button = External::Button2<b, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(longPress)>;
    
    using eeprom = EEProm::Controller<Storage::ApplData>;
    
//    using wdt   = WatchDog<systemTimer::intervall>;
};

template<typename Timer, typename Rot, typename But,
         typename SBus, typename EEProm>
struct GlobalFSM {
    enum class State : uint8_t {RunAbsolute, 
                                RunBounded,
                                RunSpeed, 
                                Undefined, Init,
                                Config, ConfigAbsolute, ConfigBounded, ConfigSpeed};
    
    static inline constexpr External::Tick<Timer> initTicks{100_ms};
    static inline constexpr External::Tick<Timer> speedTicks{200_ms};
    static inline constexpr External::Tick<Timer> eepromTicks{500_ms};
    
    using rot1_t = Meta::nth_element<0, typename Rot::type_list>;
    using rot2_t = Meta::nth_element<1, typename Rot::type_list>;
    
    using sbus = SBus;
    using sbus_pa = sbus::pa_t;
    
    using eeprom = EEProm;
    
    inline static void init() {
        Rot::init();
        But::init();

        sbus::init(); 
//        sbus::usart::template rxEnable<false>();
        
        eeprom::init();
        if (eeprom::data().magic() != 42) {
            eeprom::data().clear();
        }
    }  
    inline static void periodic() {
        sbus::periodic(); 
        eeprom::saveIfNeeded();
    }  
     
    inline static void setOutputs(const auto rot) {
        const auto v = etl::scaleTo<typename sbus::value_type>(rot);
        sbus::template set<15>(v);
        if (const auto v = sbus_pa::value(15); v) {
            sbus::template set<14>(v.toRanged());
        }
        std::byte s{};
        if (std::any(sbus_pa::switches() & External::SBus::ch18)) {
            s = External::SBus::ch17;
        }
        if (But::pressed()) {
            s |= External::SBus::ch18;
        }
        sbus::switches(s);
    }
    inline static void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTicks;
        ++mSpeedTicks;
        
        (++mEEpromTicks).on(eepromTicks, []{
            eeprom::data().expire(); 
        });
        
        sbus::ratePeriodic();
        Rot::rateProcess();
        But::ratePeriodic();
        
        const auto be = But::event();
        
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if (But::pressed()) {
                mState = State::Config;
            }
            else {
                mStateTicks.on(initTicks, []{
                    switch(eeprom::data().rotation()) {
                    case Storage::Rotation::Absolute:
                        mState = State::RunAbsolute;
                        break;
                    case Storage::Rotation::Bounded:
                        mState = State::RunBounded;
                        break;
                    case Storage::Rotation::Speed:
                        mState = State::RunSpeed;
                        break;
                    }
                });
            }
            break;
        case State::Config:
            if (be == But::Press::Short) {
                mState = State::ConfigAbsolute;            
            }
            break;
        case State::ConfigAbsolute:
            if (be == But::Press::Short) {
                mState = State::ConfigBounded;            
            }
            else if (be == But::Press::Long) {
                eeprom::data().rotation() = Storage::Rotation::Absolute;
                eeprom::data().changed();
                mState = State::RunAbsolute;
            }
            break;
        case State::ConfigBounded:
            if (be == But::Press::Short) {
                mState = State::ConfigSpeed;            
            }
            else if (be == But::Press::Long) {
                eeprom::data().rotation() = Storage::Rotation::Bounded;
                eeprom::data().changed();
                mState = State::RunBounded;
            }
            break;
        case State::ConfigSpeed:
            if (be == But::Press::Short) {
                mState = State::ConfigAbsolute;            
            }
            else if (be == But::Press::Long) {
                eeprom::data().rotation() = Storage::Rotation::Speed;
                eeprom::data().changed();
                mState = State::RunSpeed;                
            }
            break;
        case State::RunAbsolute:
            Rot::value().visit([&]<typename T>(const T vv){
                                   setOutputs(vv);
                               });
            break; 
        case State::RunBounded:
            Rot::value().visit([&]<typename T>(const T vv){
                                   setOutputs(vv);
                               });
            break; 
        case State::RunSpeed:
            static uint8_t last{127}; 
            mSpeedTicks.on(speedTicks, [&]{
                Rot::value().visit([&]<typename T>(const T vv){
                                       const uint8_t diff = 4 * (vv - last) + 127;
                                       last = vv;
                                       setOutputs(diff);
                                   });
            });
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                break;
            case State::RunAbsolute:
                Rot::set(rot1_t{127});
                break;
            case State::RunSpeed:
                Rot::set(rot1_t{127});
                break;
            case State::RunBounded:
                Rot::set(rot2_t{127}); // activate other variante type
                break;
            case State::Config:
                sbus::template set<15>(typename sbus::value_type(sbus::sbus_max));
                break;
            case State::ConfigAbsolute:
                sbus::template set<15>(typename sbus::value_type(sbus::sbus_mid + (sbus::sbus_max - sbus::sbus_min) / 4));
                break;
            case State::ConfigBounded:
                sbus::template set<15>(typename sbus::value_type(sbus::sbus_mid + 2 * (sbus::sbus_max - sbus::sbus_min) / 4));
                break;
            case State::ConfigSpeed:
                sbus::template set<15>(typename sbus::value_type(sbus::sbus_mid + 3 * (sbus::sbus_max - sbus::sbus_min) / 4));
                break;
            }
        }
    }  
private:
    static inline External::Tick<Timer> mStateTicks;
    static inline External::Tick<Timer> mSpeedTicks;
    static inline External::Tick<Timer> mEEpromTicks;
    
    inline static State mState{State::Undefined};
};

template<typename Devs>
struct Application {
    using gfsm = GlobalFSM<typename Devs::systemTimer, typename Devs::rotary, typename Devs::button, typename Devs::sbus, typename Devs::eeprom>;
    
    inline static void init() {
        Devs::portmux::init();
        Devs::ccp::unlock([]{
            Devs::clock::template prescale<1>(); 
        });
        Devs::systemTimer::init();
        gfsm::init();
    }  
    
    [[noreturn]] inline static void run() {
        while(true) {
            gfsm::periodic();
            Devs::systemTimer::periodic([&]{
                gfsm::ratePeriodic();
            });
        }    
    }
};

using devices = Devices<>;
using app = Application<devices>;

int main() {
    app::init();
    app::run();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    while(true) {
    }
}
template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif

