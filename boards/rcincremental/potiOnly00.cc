#define NDEBUG

// Nur Poti + Taster

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
//#include <external/ibus/ibus.h>
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

inline static constexpr auto fRtc = 2000_Hz;
inline static constexpr auto longPress = 3000_ms;

template<typename Out, typename In>
static inline Out enum_cast(const In in) {
    return Out{(uint8_t)in};    
}

namespace Storage {
    enum class Rot : uint8_t {Absolute = 1, Bounded, Speed};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        
        uint8_t magic() {
            return mMagic;
        }
        void clear() {
            mMagic = 42;
            mRot = Rot::Absolute;
            change();
        }
        Rot& rotation() {
            return mRot;
        }
    private:
        Rot mRot{Rot::Absolute};
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

    using dac = DAC<0>; // VRef = 2,5V
    using outPin = Pin<Port<A>, 1>; // Tastausgang
    
    using portmux = Portmux::StaticMapper<Meta::List<tca0Position>>;
    
    using pinA = Pin<Port<A>, 3>;
    using pinB = Pin<Port<A>, 2>;

    using rot1_t = uint8_t;
    using rot2_t = etl::uint_ranged<uint8_t, 0, 255>;
    using rotary = External::RotaryEncoder<pinA, pinB, std::variant<rot1_t, rot2_t>>;
    
    using pinT = Pin<Port<A>, 7>;
    using b = ActiveLow<pinT, Input>;
    using button = External::Button2<b, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(longPress)>;

    using eeprom = EEProm::Controller<Storage::ApplData>;
};

template<typename Timer, typename Rot, typename But, typename Dac, typename OutPin, typename EEProm>
struct GlobalFSM {
    enum class State : uint8_t {RunAbsolute = (uint8_t)Storage::Rot::Absolute, 
                                RunBounded = (uint8_t)Storage::Rot::Bounded,
                                RunSpeed = (uint8_t)Storage::Rot::Speed, 
                                Undefined, Init};
    
    static inline constexpr External::Tick<Timer> initTicks{100_ms};
    static inline constexpr External::Tick<Timer> speedTicks{200_ms};
    static inline constexpr External::Tick<Timer> eepromTicks{300_ms};
    
    using rot1_t = Meta::nth_element<0, typename Rot::type_list>;
    using rot2_t = Meta::nth_element<1, typename Rot::type_list>;

    using eeprom = EEProm;
    
    inline static void init() {
        Rot::init();
        But::init();
        eeprom::init();
        if (eeprom::data().magic() != 42) {
            eeprom::data().clear();
        }
        Dac::init();        
        OutPin::low();
        OutPin::template dir<Output>();
    }  
    inline static void periodic() {
        eeprom::saveIfNeeded();
    }  
    inline static void setOutputs(const uint8_t rot) {
        Dac::put(rot);
        if (But::pressed()) {
            OutPin::high();
        }
        else {
            OutPin::low();
        }
    }
    inline static void ratePeriodic() {
        Rot::rateProcess();
        But::ratePeriodic();
        
        ++mStateTicks;
        ++mSpeedTicks;
        (++mEEpromTicks).on(eepromTicks, []{
            eeprom::data().expire(); 
        });
        
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if (But::pressed()) {
                const auto storedRot = eeprom::data().rotation();
                switch(storedRot) {
                case Storage::Rot::Absolute:
                    mState = State::RunBounded;
                    eeprom::data().rotation() = Storage::Rot::Bounded; 
                    eeprom::data().change();
                    break;
                case Storage::Rot::Bounded:
                    mState = State::RunSpeed;
                    eeprom::data().rotation() = Storage::Rot::Speed; 
                    eeprom::data().change();
                    break;
                case Storage::Rot::Speed:
                    mState = State::RunAbsolute;
                    eeprom::data().rotation() = Storage::Rot::Absolute; 
                    eeprom::data().change();
                    break;
                }
            }
            else {
                mStateTicks.on(initTicks, []{
                    mState = enum_cast<State>(eeprom::data().rotation());
                });
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
            mSpeedTicks.on(speedTicks, [&]{
                static uint8_t last = 127; // why warning?
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
            }
        }
    }  
private:
    static inline External::Tick<Timer> mStateTicks;
    static inline External::Tick<Timer> mSpeedTicks;
    static inline External::Tick<Timer> mEEpromTicks;
    inline static State mState{State::Undefined};
    inline static bool mButton{};
};

template<typename Devs>
struct Application {
    using dac = typename Devs::dac;
    using outPin = typename Devs::outPin;
    using gfsm = GlobalFSM<typename Devs::systemTimer, typename Devs::rotary, typename Devs::button, dac, outPin, typename Devs::eeprom>;
    
    inline static void init() {
        Devs::portmux::init();
        Devs::ccp::unlock([]{
            Devs::clock::template prescale<4>(); // 5MHz
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

