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
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>

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
#include <external/solutions/series01/rpm.h>
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
#endif
    constexpr uint16_t R1vd = 10'000;
    constexpr uint16_t R2vd = 2'000;
}

template<typename R>
struct RpmProvider {
#ifdef FS_I6S    
    inline static constexpr auto ibus_type = IBus::Type::type::RPM_FLYSKY;
#else
    inline static constexpr auto ibus_type = IBus::Type::type::RPM;
#endif
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        auto rpm = R::value();
        if (rpm) {
            return rpm.value();
        }
        return 0;
    }
};

template<typename Sensor>
struct VoltageProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::EXTERNAL_VOLTAGE;
    inline static constexpr void init() {}
    
    inline static constexpr uint16_t value() {
        return Sensor::value();
    }
};
template<typename Sensor>
struct CurrentProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
    inline static constexpr void init() {}
    
    inline static constexpr uint16_t value() {
        return Sensor::value();
    }
};

template<typename ADC, uint8_t Channel, typename SigRow>
struct InternalTempProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return SigRow::template adcValueToTemperature<std::ratio<1,10>, 0>(ADC::value(channel)).value;
    }
};

template<typename ADC, uint8_t Channel>
struct Mcp9700aProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        auto raw = ADC::value(channel);
        if (raw.isTop()) {
            return 0;
        }
        else {
            auto v = raw.toInt(); 
            v *= 34;
            v >>= 3;
            if (v >= 100) {
                return v - 100;
            }
            else {
                return 0;
            }
        }
    }
};

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
 
template<typename Timer, typename PWM, typename PA>
struct EscFsm {
    enum class State : uint8_t {Undefined = 0, Init, 
                                Off = 10, 
                                Forward = 20, ForwardWait, 
                                Backward = 30, BackwardWait};

    enum class Event : uint8_t {None, Start};
    
    using throttle_type = typename PA::value_type;
    using thr_t = typename throttle_type::value_type;
//    thr_t::_;
    
    using pwm_t = PWM::value_type;
//    pwm_t::_;

    static inline constexpr thr_t thrMax{throttle_type::Upper};
    static inline constexpr thr_t thrMin{throttle_type::Lower};
    static inline constexpr thr_t thrMedium{(throttle_type::Upper + throttle_type::Lower) / 2};
    
    static inline constexpr thr_t thrStartForward{thrMedium + 20};
    static inline constexpr thr_t thrStartBackward{thrMedium - 20};
    
    static constexpr External::Tick<Timer> waitTicks{100_ms}; // fw <-> bw
    static constexpr External::Tick<Timer> deadTicks{500_ms}; // fw <-> bw
    
    inline static void init() {
        PWM::init();
        PWM::period(mPwmPeriod);
        PWM::template off<Meta::List<AVR::PWM::WO<0>>>();
        PWM::template off<Meta::List<AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
    }
    
    inline static void event(const Event e) {
        mEvent = e;
    }
    inline static Event event() {
        Event e{Event::None};
        std::swap(e, mEvent);
        return e;
    }
    inline static bool isThrottleValid() {
        return !!mThrottle;
    }
    inline static bool isThrottleForward() {
        return mThrottle && (mThrottle.toInt() >= thrStartForward);
    }
    inline static bool isThrottleBackward() {
        return mThrottle && (mThrottle.toInt() <= thrStartBackward);
    }
    inline static bool isThrottleOn() {
        return isThrottleBackward() || isThrottleForward();
    }
    inline static void ratePeriodic() {
        mThrottle = PA::value(0);
        ++mStateTicks;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(waitTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if (const auto e = event(); e == Event::Start) {
                mState = State::Off;
            }
            break;
        case State::ForwardWait:
            mStateTicks.on(deadTicks, []{
                mState = State::Forward;
            });
            if (!mThrottle || (mThrottle.toInt() < thrStartForward)) {
                mState = State::Off;
            }
            break;
        case State::Forward:
            if (const auto tv = mThrottle.toInt(); !mThrottle || (tv < thrStartForward)) {
                mState = State::Off;
            }
            else {
                const auto d = etl::distance(tv, thrStartForward);
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax - thrStartForward}, etl::Intervall{pwm_t{0}, mPwmPeriod});
                PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(pv);
            }
            break;
        case State::Off:
            if (mThrottle && (mThrottle.toInt() > thrStartForward)) {
                mState = State::ForwardWait;
            }
            else if (mThrottle && (mThrottle.toInt() < thrStartBackward)) {
                mState = State::BackwardWait;
            } 
            break;
        case State::BackwardWait:
            mStateTicks.on(deadTicks, []{
                mState = State::Backward;
            });
            if (!mThrottle || (mThrottle.toInt() > thrStartBackward)) {
                mState = State::Off;
            }
            break;
        case State::Backward:
            if (const auto tv = mThrottle.toInt(); !mThrottle || (tv > thrStartBackward)) {
                mState = State::Off;
            }
            else {
                const auto d = etl::distance(tv, thrStartBackward);
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward - thrMin}, etl::Intervall{pwm_t{0}, mPwmPeriod});
                PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(pv);
            }
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                off();
                break;
            case State::ForwardWait:
                forward();
                break;
            case State::Forward:
                forward();
                break;
            case State::Off:
                off();
                break;
            case State::BackwardWait:
                backward();
                break;
            case State::Backward:
                backward();
                break;
            }   
        }
    }    
//private:
    inline static void off() {
        PWM::template off<Meta::List<AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
    }
    inline static void forward() {
        PWM::template off<Meta::List<AVR::PWM::WO<2>>>();
        PWM::template on<Meta::List<AVR::PWM::WO<1>>>();
    }
    inline static void backward() {
        PWM::template off<Meta::List<AVR::PWM::WO<1>>>();
        PWM::template on<Meta::List<AVR::PWM::WO<2>>>();
    }
    inline static PWM::value_type mPwmPeriod{60000};
    static inline throttle_type mThrottle;
    inline static External::Tick<Timer> mStateTicks;
    inline static State mState{State::Undefined};
    inline static Event mEvent{Event::None};
};

template<typename Timer, typename Servo, typename Led, typename Esc, 
         typename Sensor, typename Lut, 
         typename Adc, typename Rpm,
         typename Term = void>
struct GlobalFsm {
    using TermDev = Term::device_type;
    
    enum class State : uint8_t {Undefined, Init, SignalWait, CheckStart, Run};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> initTicks{500_ms};
    static constexpr External::Tick<Timer> debugTicks{500_ms};
    static constexpr External::Tick<Timer> resetTicks{2000_ms};
    static constexpr External::Tick<Timer> learnTimeoutTicks{4000_ms};
    
    inline static void init() {
        if constexpr (std::is_same_v<Servo, TermDev>) {
            Servo::template init<AVR::BaudRate<115200>>();            
        }
        else {
            Servo::template init<AVR::BaudRate<115200>>();
            if constexpr(!std::is_same_v<TermDev, void>) {
                TermDev::template init<AVR::BaudRate<9600>>();
            }
        }
        Led::init();
        Esc::init();
 
        //    lut1::init(std::byte{0xcc}); // route TXD to lut1-out no-inv
        //    lut1::init(std::byte{0x33}); // route TXD to lut1-out inv
        Lut::init(std::byte{0x00}); 
        Sensor::init();
        Sensor::uart::txOpenDrain();
        
        Adc::template init<true>(); // pullups
        Adc::mcu_adc_type::nsamples(6);
        
        Rpm::init();
    }
    inline static void periodic() {
        Servo::periodic();
        Led::periodic();
        Sensor::periodic();
        Adc::periodic();
//        Rpm::periodic();
    }
    inline static void ratePeriodic() {
        Sensor::ratePeriodic();
        Esc::ratePeriodic();
        const auto oldState = mState;
        ++mStateTick;
        ++mDebugTick;
        ++mResetTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                mState = State::SignalWait;
            });
            break;
        case State::SignalWait:
            if (Esc::isThrottleValid()) {
                mState = State::CheckStart;
            }
            break;
        case State::CheckStart:
            if (!(Esc::isThrottleForward() || Esc::isThrottleBackward())) {
                    mState = State::Run;
            }
            break;
        case State::Run:
            mDebugTick.on(debugTicks, debug);
            mResetTick.on(resetTicks, []{
                Rpm::reset();
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<Term>("init"_pgm);
                led(red);
                break;
            case State::SignalWait:
                etl::outl<Term>("sigw"_pgm);
                led(yellow);
                break;
            case State::CheckStart:
                etl::outl<Term>("check"_pgm);
                led(magenta);
                break;
            case State::Run:
                etl::outl<Term>("run"_pgm);
                Esc::event(Esc::Event::Start);
                led(green);
                break;
            }
        }
    }
private:
    static inline void debug() {
        etl::outl<Term>("t: "_pgm, Esc::mThrottle.toInt(), " s: "_pgm, (uint8_t)Esc::mState, 
                        " r: "_pgm, Rpm::value());
    }
    
    using Crgb = External::Crgb;
    static constexpr Crgb red{External::Red{255}, External::Green{0}, External::Blue{0}};
    static constexpr Crgb green{External::Red{0}, External::Green{255}, External::Blue{0}};
    static constexpr Crgb blue{External::Red{0}, External::Green{0}, External::Blue{255}};
    static constexpr Crgb yellow{External::Red{255}, External::Green{255}, External::Blue{0}};
    static constexpr Crgb magenta{External::Red{255}, External::Green{0}, External::Blue{255}};
    static constexpr Crgb cyan{External::Red{0}, External::Green{255}, External::Blue{255}};
    
    inline static void led(const Crgb& c) {
        using index_t = Led::index_type;
        Led::set(index_t{0}, c);
        Led::out();
    }
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mDebugTick;
    static inline External::Tick<Timer> mResetTick;
};

struct VersionProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Temp1;
    inline static constexpr auto ibus_type = IBus::Type::type::ARMED;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return 1001;
    }
};

#ifndef NDEBUG
using assertPin = Pin<Port<A>, 0>; 
#endif

using timing0Pin = Pin<Port<D>, 3>; 
using timing1Pin = Pin<Port<D>, 4>; 
using timing2Pin = Pin<Port<D>, 5>; 

using daisyChain= Pin<Port<A>, 7>; 

using inh1Pin = Pin<Port<A>, 3>; 
using inh2Pin = Pin<Port<A>, 4>; 

using rpmPin = Pin<Port<F>, 2>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Sensor
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Servo / Debug

// lut1 out: pc3
using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;
using lut1 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Usart<1>, Ccl::Input::Mask>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;

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
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1, 6, 7, 19, 20, 21, 0x42>>; // 42 = temp

using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;
using spi = AVR::Spi<spiPosition, AVR::QueueLength<16>,  AVR::UseInterrupts<false>>;
using led = External::LedStripe<spi, External::APA102, 1>;

using rpmPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using rpm = External::Rpm::RpmFreq<tcaPosition::component_type, rpmPosition::component_type>;
using evch0 = Event::Channel<4, Event::Generators::Pin<rpmPin>>;
using evuser0  = Event::Route<evch0, Event::Users::TcbCapt<0>>;

using temp1P = Mcp9700aProvider<adcController, 0>;
using textP  = Mcp9700aProvider<adcController, 4>;
using temp2P = Mcp9700aProvider<adcController, 5>;
using tempiP = InternalTempProvider<adcController, 7, sigrow>;

using vdiv = External::AnalogSensor<adcController, 3, std::ratio<0,1>, 
                                    std::ratio<R2vd, R2vd + R1vd>, 
                                    std::ratio<100,1>>;
using voltageP = VoltageProvider<vdiv>;

using bec1S = External::AnalogSensor<adcController, 1, std::ratio<0,1>, 
                                    std::ratio<24000, 1000>, 
                                    std::ratio<100,1>>;
using cBecP = CurrentProvider<bec1S>;

using currS = External::AnalogSensor<adcController, 6, std::ratio<0,1>, 
                                    std::ratio<1900, 1000>, 
                                    std::ratio<100,1>>;
using currP = CurrentProvider<currS>;

using rpmP = RpmProvider<rpm>;

using sensor = IBus::Sensor<usart1Position, AVR::Usart, AVR::BaudRate<115200>, 
                            Meta::List<VersionProvider, 
                                       temp1P, temp2P, tempiP, textP, 
                                       voltageP, cBecP, currP, rpmP>, 
                            systemTimer, ibt
//                          , etl::NamedFlag<true>
//                           , etl::NamedFlag<true>
>;

using evrouter = Event::Router<Event::Channels<evch0>, Event::Routes<evuser0>>;
 
using escfsm = EscFsm<systemTimer, pwm, servo_pa>;
using gfsm = GlobalFsm<systemTimer, servo, led, escfsm, sensor, lut1, adcController, rpm, terminal>;

using portmux = Portmux::StaticMapper<Meta::List<spiPosition, ccl1Position, tcaPosition, usart1Position, usart2Position, rpmPosition>>;

int main() {
    timing0Pin::dir<Output>();
    timing1Pin::dir<Output>();
    timing2Pin::dir<Output>();
    
    portmux::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();
    
    inh1Pin::dir<Output>();
    inh1Pin::on();
    inh2Pin::dir<Output>();
    inh2Pin::on();
    
    evrouter::init();
    gfsm::init();
    
    {
        etl::outl<terminal>("test01"_pgm);
        
        while(true) {
            timing0Pin::toggle();
            
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
