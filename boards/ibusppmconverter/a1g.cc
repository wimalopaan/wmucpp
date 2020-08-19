#define NDEBUG

#define USE_IBUS

#include "board.h"
#include "temp.h"
#include "gps.h"

using adcController = External::Hal::AdcController<adc, Meta::NList<0x1e>>; // 0x1e = temp


using ibus_full = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<speedP, bytesP, packagesR, packagesV>, 
                          systemTimer, ibt>;

using ibus_gps = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<speedP>, 
                          systemTimer, ibt>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using isrRegistrar = IsrRegistrar<typename gpsUsart::StartBitHandler, typename gpsUsart::BitHandler>;

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

    q0Pin::pullup<true>();
    q0Pin::template dir<Input>();
    
    for(uint16_t i = 0; i < 1000; ++i) {
        q0Pin::read();        
    }
    
    const bool useAllProviders = !q0Pin::read(); // pull low to use all providers
    
    if (useAllProviders) {
        ibus_full::init();
    }
    else {
        ibus_gps::init();
    }
    gpsUsart::init<AVR::HalfDuplex>();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
    const auto resetTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        if (useAllProviders) {
            ibus_full::periodic();
        }
        else {
            ibus_gps::periodic();
        }
        adcController::periodic();
        
        systemTimer::periodic([&]{
            wdt::reset();
            if (useAllProviders) {
                ibus_full::ratePeriodic();
            }
            else {
                ibus_gps::ratePeriodic();
            }
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                }
                else if (resetTimer == t) {
                }
            });
        });
    }
}

ISR(PORTA_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<rxPin::name_type>>();
}

ISR(TCD0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tcd<0>::Ovf>();
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
