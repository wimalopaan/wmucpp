#define USE_IBUS

#include "board.h"
#include "temp.h"
#include "curr.h"

using dbg1 = so4Pin;

using adcController = External::Hal::AdcController<adc, Meta::NList<10, 11, 5, 3, 0x1e>>; // 1e = temp
using mcp0P = Mcp9700aProvider<adcController, 0>;
using mcp1P = Mcp9700aProvider<adcController, 1>;

using curr0P = ACS723U40Provider<adcController, 2>;
using curr1P = ACS723U40Provider<adcController, 3>;


using iTempP = InternalTempProvider<adcController, 4>;

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<mcp0P, mcp1P, curr0P, curr1P>, 
                          systemTimer, ibt>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

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
    
    portmux::init();
    systemTimer::init();
    adcController::init();
    
    ibus::init();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    while(true) {
        ibus::periodic();
        adcController::periodic();
        
        systemTimer::periodic([&]{
            wdt::reset();
            ibus::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
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
