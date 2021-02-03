//#define NDEBUG

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

#include <external/solutions/cells.h>
#include <external/ibus/ibus.h>
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

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

inline static constexpr auto fRtc = 2000_Hz;

template<typename HWRev = void>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    using sleep = Sleep<>;
    
    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
    using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;
    
    using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

    using dac = DAC<0>;
    
    using pinA = Pin<Port<A>, 3>;
    using pinB = Pin<Port<A>, 2>;
    
    using rot_t = uint8_t;
    using rotary = External::RotaryEncoder<pinA, pinB, rot_t, rot_t{127}>;

    using pinT = Pin<Port<A>, 7>;
    using b = ActiveLow<pinT, Input>;
    using button = External::Button<b, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(1000_ms)>;
    
};

template<typename Timer, typename Rot, typename But, typename Dac, typename TermDev>
struct GlobalFSM {
    enum class State : uint8_t {Undefined, Init, RunAbsolute, RunSpeed};
    
    static inline constexpr External::Tick<Timer> initTicks{100_ms};
    static inline constexpr External::Tick<Timer> debugTicks{500_ms};
    static inline constexpr External::Tick<Timer> speedTicks{200_ms};
    
    using terminal = etl::basic_ostream<TermDev>;

    inline static void init() {
        Rot::init();
        But::init();
        Dac::init();        
        Dac::put(127);
    }  
    inline static void periodic() {
        TermDev::periodic(); 
        
    }  
    inline static void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTicks;
        ++mSpeedTicks;
        (++mDebugTicks).on(debugTicks, debug);
        
        Rot::rateProcess();
        But::periodic();
        
        const auto be = But::event();
        
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTicks.on(initTicks, []{
                mState = State::RunAbsolute;
            });
            break;
        case State::RunAbsolute:
        {
            if (be == But::Press::Long) {
                mState = State::RunSpeed;
                But::reset();
            }
            const auto v = Rot::value();
            Dac::put(v);
        }
            break;
        case State::RunSpeed:
        {
            if (be == But::Press::Long) {
                mState = State::RunAbsolute;
                But::reset();
            }
            mSpeedTicks.on(speedTicks, [&]{
                static uint8_t last = 127;
                const int8_t v = 4 * (Rot::value() - last);
                last = Rot::value();
                Dac::put(127 + v);
            });
        }
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("s i"_pgm);
                break;
            case State::RunAbsolute:
                etl::outl<terminal>("s ra"_pgm);
                break;
            case State::RunSpeed:
                etl::outl<terminal>("s rs"_pgm);
                break;
            }
        }
    }  
private:
    static inline void debug() {
        etl::outl<terminal>("v: "_pgm, Rot::value());        
    }
    static inline External::Tick<Timer> mStateTicks;
    static inline External::Tick<Timer> mDebugTicks;
    static inline External::Tick<Timer> mSpeedTicks;
    inline static State mState{State::Undefined};
};

template<typename Devs>
struct Application {
    using gfsm = GlobalFSM<typename Devs::systemTimer, typename Devs::rotary, 
    typename Devs::button, typename Devs::dac, typename Devs::terminalDevice>;
    
    inline static void init() {
        Devs::portmux::init();
        Devs::ccp::unlock([]{
            Devs::clock::template prescale<1>(); 
        });
        Devs::terminalDevice::template init<AVR::BaudRate<115200>>();
        Devs::systemTimer::init();
        
        gfsm::init();
    }  
    inline static void run() {
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
