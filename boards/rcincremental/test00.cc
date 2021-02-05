//#define NDEBUG

// Drei Modi: 
// 1) Poti
// 2) Cppm-Master
// 3) Cppm-Slave
//
// Beim Einschalten Taster 10s gedrückt halten, mit LED an Out-Pin kontrollierem
// Blink-Muster

// CPPM = Master (serial receive)
#define CPPM
#ifdef CPPM
# define NDEBUG
#endif

// Serial = Slave (serial send)
//#define SERIAL
#ifdef SERIAL
# define NDEBUG
#endif

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
#include <external/solutions/series01/sppm_out.h>
#include <external/solutions/series01/cppm_out.h>

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

    using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, Portmux::Default>;
    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
#ifdef CPPM
    using cppm = External::Ppm::Cppm<tca0Position>;
    using outPin = void;
    using dac = void;
    using lut0 = Ccl::SimpleLut<0, Ccl::Input::Mask, Ccl::Input::Tca0<1>, Ccl::Input::Mask>;    
    using cell_pa = External::CellPA<0, void, IBus::CheckSum>;
    using recv_uart = Usart<usart0Position, cell_pa, UseInterrupts<false>>;
#else
# ifdef SERIAL
    using cppm = void;
    using outPin = void;
    using dac = void;
    using lut0 = void;
    using send_uart = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
# else
    using cppm = void;
    using outPin = Pin<Port<A>, 1>;
    using dac = DAC<0>;
    using lut0 = void;
# endif
#endif

# ifndef NDEBUG    
    using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
# else
    using terminalDevice = void;
# endif
    
    using portmux = Portmux::StaticMapper<Meta::List<tca0Position, usart0Position>>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

    
    using pinA = Pin<Port<A>, 3>;
    using pinB = Pin<Port<A>, 2>;

    using rot1_t = uint8_t;
    using rot2_t = etl::uint_ranged<uint8_t, 0, 255>;
    using rotary = External::RotaryEncoder<pinA, pinB, std::variant<rot1_t, rot2_t>>;
    
    using pinT = Pin<Port<A>, 7>;
    using b = ActiveLow<pinT, Input>;
    using button = External::Button<b, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(1000_ms)>;
    
};

template<typename Timer, typename Rot, typename But, typename Dac, typename OutPin, typename CPpm, typename Lut,
         typename TermDev>
struct GlobalFSM {
    enum class State : uint8_t {Undefined, Init, RunAbsolute, RunSpeed, RunBounded};
    
    static inline constexpr External::Tick<Timer> initTicks{100_ms};
    static inline constexpr External::Tick<Timer> debugTicks{500_ms};
    static inline constexpr External::Tick<Timer> speedTicks{200_ms};
    
    using terminal = etl::basic_ostream<TermDev>;

    using rot1_t = Meta::nth_element<0, typename Rot::type_list>;
    using rot2_t = Meta::nth_element<1, typename Rot::type_list>;
    
    using channel_t = typename CPpm::channel_t;
    using ch_value_t = typename CPpm::ranged_type;
    
    inline static void init() {
        Rot::init(rot1_t{127});
        But::init();
        if constexpr(!std::is_same_v<CPpm, void>) {
            CPpm::init();        
            if constexpr(!std::is_same_v<Lut, void>) {
                Lut::init(std::byte{0x33}); // invert
//                Lut::init(std::byte{0xcc}); // no invert
            }
        }
        if constexpr(!std::is_same_v<Dac, void>) {
            Dac::init();        
        }
        if constexpr(!std::is_same_v<OutPin, void>) {
            OutPin::low();
            OutPin::template dir<Output>();
        }
    }  
    inline static void periodic() {
        if constexpr(!std::is_same_v<TermDev, void>) {
            TermDev::periodic(); 
        }
        if constexpr(!std::is_same_v<CPpm, void>) {
            CPpm::periodic();        
        }
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
                mState = State::RunBounded;
                But::reset();
            }
            const auto& variant = Rot::value();
            variant.visit([]<typename T>(const T vv){
                if constexpr(!std::is_same_v<Dac, void>) {
                    Dac::put(vv);
                }
                if constexpr(!std::is_same_v<CPpm, void>) {
                                  if constexpr(std::is_same_v<T, uint8_t>) {
                                      const auto rv = etl::uint_ranged<uint8_t, 0, 255>{vv};
                                      ch_value_t v = etl::scaleTo<ch_value_t>(rv);
                                      CPpm::set(channel_t{0}, v);
                                  }
                                  else {
                                      ch_value_t v = etl::scaleTo<ch_value_t>(vv);
                                      CPpm::set(channel_t{0}, v);                                      
                                  }
                                  
                }
            });
        }
            break; 
        case State::RunBounded:
        {
            if (be == But::Press::Long) {
                mState = State::RunSpeed;
                But::reset();
            }
            const auto& variant = Rot::value();
            variant.visit([](const auto vv){
                if constexpr(!std::is_same_v<Dac, void>) {
                    Dac::put(vv);
                }
            });
        }
            break; 
        case State::RunSpeed:
        {
            if (be == But::Press::Long) {
                mState = State::RunAbsolute;
                But::reset();
            }
            mSpeedTicks.on(speedTicks, [&]{
                static uint8_t last = 127; // why warning?
                const auto& variant = Rot::value();
                variant.visit([&](const auto vv){
                    const int8_t diff = 4 * (vv - last);
                    last = vv;
                    if constexpr(!std::is_same_v<Dac, void>) {                     
                        Dac::put(127 + diff);   
                    }
                });
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
                Rot::set(rot1_t{127});
                etl::outl<terminal>("s ra"_pgm);
                break;
            case State::RunSpeed:
                Rot::set(rot1_t{127});
                etl::outl<terminal>("s rs"_pgm);
                break;
            case State::RunBounded:
                Rot::set(rot2_t{127});
                etl::outl<terminal>("s rb"_pgm);
                break;
            }
        }
    }  
private:
    static inline void debug() {
        Rot::value().visit([](const auto vv){
//            decltype(vv)::_;
            etl::outl<terminal>("v: "_pgm, vv);                    
        });
    }
    static inline External::Tick<Timer> mStateTicks;
    static inline External::Tick<Timer> mDebugTicks;
    static inline External::Tick<Timer> mSpeedTicks;
    inline static State mState{State::Undefined};
};

template<typename Devs>
struct Application {
    using gfsm = GlobalFSM<typename Devs::systemTimer, typename Devs::rotary, 
    typename Devs::button, typename Devs::dac, typename Devs::outPin, typename Devs::cppm, typename Devs::lut0,
    typename Devs::terminalDevice>;
    
    inline static void init() {
        Devs::portmux::init();
        Devs::ccp::unlock([]{
            Devs::clock::template prescale<1>(); 
        });
        if constexpr(!std::is_same_v<typename Devs::terminalDevice, void>) {
            Devs::terminalDevice::template init<AVR::BaudRate<115200>>();
        }
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
