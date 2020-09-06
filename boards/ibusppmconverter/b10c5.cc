#define USE_IBUS

#include "board.h"

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;

using tcb0Position = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppmB = External::Ppm::PpmOut<tcb0Position>;

using tcb1Position = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
using ppmC = External::Ppm::PpmOut<tcb1Position>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcb0Position, tcb1Position>>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using evch0 = Event::Channel<0, void>;
using evch1 = Event::Channel<1, void>;
using evuser0 = Event::Route<evch0, Event::Users::Tcb<0>>;
using evuser1 = Event::Route<evch1, Event::Users::Tcb<1>>;
using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;

int main() {
    wdt::init<ccp>();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    reset::noWatchDog([]{
        uninitialzed::reset();
    });
    
    reset::onWatchDog([]{
        uninitialzed::counter = uninitialzed::counter + 1;        
    });
    
    evrouter::init();
    
    portmux::init();
    systemTimer::init();
    
    servo::init<AVR::BaudRate<115200>, HalfDuplex>();
    servo::txEnable<false>();
    
    ppmA::init();
    ppmB::init();
    ppmC::init();
    
    const auto periodicTimer = alarmTimer::create(20_ms, External::Hal::AlarmFlags::Periodic);

    using index_t = ppmA::index_type;
    using channel_t = servo_pa::channel_t;
//    using value_t = servo_pa::value_type;

    constexpr uint8_t offset{9}; // ch 10
    etl::uint_ranged_circular<uint8_t, 0, 4> index;

    while(true) {
        servo::periodic();
        
        auto cv = servo_pa::value(channel_t(index + offset));
        
        if (index == 0) { // SO1
            ppmA::ppm(index_t{1}, cv);
        }
        else if (index == 1) { // SO2
            ppmA::ppm(index_t{0}, cv);
        }
        else if (index == 2) { // SO3
            ppmB::ppm_async(cv);
        }
        else if (index == 3) { // SO4
            ppmC::ppm_async(cv);
        }
        else if (index == 4) { // Q0
            ppmA::ppm(index_t{2}, cv);
        }
        ++index;
        systemTimer::periodic([&]{
            wdt::reset();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    ppmB::onReload([]{
                        evrouter::strobe<0>();
                    });
                    ppmC::onReload([]{
                        evrouter::strobe<1>();
                    });
                }
            });
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
