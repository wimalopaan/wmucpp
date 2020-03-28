#define NDEBUG

#define USE_IBUS

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/ibus/ibus.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/series01/sppm_out.h>

#include <external/solutions/debug/logic_analyzer.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr auto fRtc = 2048_Hz;
}

using systemTimer = SystemTimer<Component::Pit<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using ledPin    = ActiveHigh<Pin<Port<F>, 5>, Output>;
using assertPin = ActiveHigh<Pin<Port<F>, 6>, Output>;

using evPin    = Pin<Port<B>, 2>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // I-Bus Servo
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>; // Terminal

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;

using tcbPosition = Portmux::Position<Component::Tcb<2>, Portmux::Default>;
using ppmB = External::Ppm::PpmOut<tcbPosition>;

using dbg = PinGroup<Meta::List<Pin<Port<A>, 3>, Pin<Port<A>, 4>, Pin<Port<A>, 5>, Pin<Port<A>, 6>, Pin<Port<A>, 7>> >;
using tParallelDebug = External::Debug::TriggeredParallel<Pin<Port<A>, 2>, dbg>;

using terminalDevice = AVR::Usart<usart2Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = AVR::Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using portmux = Portmux::StaticMapper<Meta::List<usart1Position, usart2Position, tcaPosition, tcbPosition>>;

using evch = Event::Channel<0, Event::Generators::Tca0<AVR::Event::Generators::Kind::Ovf>>;
using evuser = Event::Route<evch, Event::Users::Tcb<2>>;
using evuser2 = Event::Route<evch, Event::Users::EvOut<B>>;
using evrouter = Event::Router<Event::Channels<evch>, Event::Routes<evuser, evuser2>>;

int main() {
    assertPin::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    evrouter::init();
    
    ledPin::init();
    portmux::init();
    systemTimer::init();

    evPin::template dir<Output>();
    
    terminalDevice::init<AVR::BaudRate<9600>>();
    servo::init<AVR::BaudRate<115200>>();
    servo::txEnable<false>();
    
    ppmA::init();
    ppmB::init();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        etl::outl<terminal>("test50"_pgm);
        
        using index_t = ppmA::index_type;
        using channel_t = servo_pa::channel_t;
        
        while(true) {
            servo::periodic();
            terminalDevice::periodic();
            
            channel_t in_ch;
            index_t out_ch;
            
            out_ch.set(0);
            in_ch.set(10);
            
            ppmA::ppm(out_ch, servo_pa::value(in_ch));
            ++out_ch;
            ++in_ch;
            ppmA::ppm(out_ch, servo_pa::value(in_ch));
//            ++out_ch;
//            ++in_ch;
//            ppmA::ppm(out_ch, servo_pa::value(in_ch));
//            ++out_ch;
//            ++in_ch;
//            ppmB::ppm(servo_pa::value(in_ch));
            
            systemTimer::periodic([&]{
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        ledPin::toggle();
                        etl::out<terminal>("c10:"_pgm); 
                        for(channel_t i{10}; i < 14; ++i) {
                            etl::out<terminal>(" "_pgm, servo_pa::value(i).toInt()); 
                        }
                        etl::outl<terminal>(" *"_pgm); 
                    }
                });
            });
        }
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    while(true) {
        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
