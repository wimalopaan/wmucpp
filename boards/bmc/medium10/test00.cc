//#define NDEBUG

#define USE_IBUS

//#define USE_SBUS
//#define USE_SPORT

#define LEARN_DOWN

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

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/apa102.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SBUS
    constexpr auto fRtc = 1000_Hz;
#endif
#ifdef USE_IBUS
    constexpr auto fRtc = 1000_Hz;
    //    constexpr auto fRtc = 2000_Hz;
#endif
}

template<typename Pin>
struct IBusThrough {
    inline static void init() {
        Pin::template dir<Output>();
    }
    inline static void on() {
        Pin::on();
    }
    inline static void off() {
        Pin::off();
    }
};

template<typename Timer, typename PWM>
struct EscFsm {
    enum class State : uint8_t {Undefined, Init, Forward, Off, Backward};

    static constexpr External::Tick<Timer> deadTicks{500_ms}; // fw <-> bw
    
    inline static void init() {
    }
    inline static void periodic() {
    }
    inline static void ratePeriodic() {
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            break;
        case State::Forward:
            break;
        case State::Off:
            break;
        case State::Backward:
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
                break;
            }   
        }
    }    
private:
    inline static State mState{State::Undefined};
};

template<typename Timer, typename Led, typename Esc, typename Term = void>
struct GlobalFsm {
    enum class State : uint8_t {Undefined, Init};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> learnTimeoutTicks{4000_ms};
    
    inline static void init() {
        Led::init();
        Esc::init();
    }
    inline static void periodic() {
        Led::periodic();
        Esc::periodic();
    }
    inline static void ratePeriodic() {
        Esc::ratePeriodic();
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                led(color_t{0});
                break;
            }
        }
    }
private:
    static constexpr External::Crgb red{External::Red{255}, External::Green{0}, External::Blue{0}};
    static constexpr External::Crgb green{External::Red{0}, External::Green{255}, External::Blue{0}};
    static constexpr External::Crgb blue{External::Red{0}, External::Green{0}, External::Blue{255}};
    static constexpr External::Crgb c1{External::Red{255}, External::Green{255}, External::Blue{0}};
    static constexpr External::Crgb c2{External::Red{255}, External::Green{0}, External::Blue{255}};
    static constexpr External::Crgb c3{External::Red{0}, External::Green{255}, External::Blue{255}};
    
    using colors_t = std::array<External::Crgb, 6>; 
    inline static constexpr colors_t colors{red, c1, green, c2, blue, c3};
    using color_t = etl::uint_ranged<uint8_t, 0, colors_t::size() - 1>;
    
    inline static void led(const color_t c) {
        using index_t = Led::index_type;
        Led::set(index_t{0}, colors[c]);
    }
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
};

#ifndef NDEBUG
using assertPin = Pin<Port<A>, 0>; 
#endif

using daisyChain= Pin<Port<A>, 7>; 

using inh1Pin = Pin<Port<A>, 3>; 
using inh2Pin = Pin<Port<A>, 4>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Sensor
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Servo / Debug

// lut1 out: pc3
using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;
using lut1 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Usart<1>, Ccl::Input::Mask>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<ccl1Position, tcaPosition, usart1Position, usart2Position>>;

// WO0: unbenutzt
// WO1: pwm1
// WO2: pwm2
using pwm = PWM::DynamicPwm<tcaPosition>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = Usart<usart2Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifndef NDEBUG
using terminal = etl::basic_ostream<servo>;
#else
using terminal = etl::basic_ostream<void>;
#endif

using ibt = IBusThrough<daisyChain>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
// ADC Channels: T1, BecI1, BecI2, V+, Text, T2, Curr
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1, 6, 7, 13, 14, 15, 0x1e>>; // 1e = temp

using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;
using spi = AVR::Spi<spiPosition, AVR::QueueLength<1>,  AVR::UseInterrupts<false>>;
using led = External::LedStripe<spi, External::APA102, 1>;

using sensor = IBus::Sensor<usart1Position, AVR::Usart, AVR::BaudRate<115200>, Meta::List<>, systemTimer, ibt
//                          , etl::NamedFlag<true>
//                           , etl::NamedFlag<true>
>;

using escfsm = EscFsm<systemTimer, pwm>;
using gfsm = GlobalFsm<systemTimer, led, escfsm, terminal>;

int main() {
    portmux::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();
    gfsm::init();
    
//    lut1::init(std::byte{0xcc}); // route TXD to lut1-out no-inv
    lut1::init(std::byte{0x33}); // route TXD to lut1-out inv
    
    {
        etl::outl<terminal>("test00"_pgm);
        
        while(true) {
            gfsm::periodic();
            systemTimer::periodic([&]{
                gfsm::ratePeriodic();
            });
        }
    }
}


#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
