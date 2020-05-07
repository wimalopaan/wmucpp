#define USE_IBUS

#include "board.h"
#include "rpm.h"
#include "temp.h"

#include <external/solutions/series01/swuart.h>
#include <external/solutions/gps.h>

//using adcController = External::Hal::AdcController<adc, Meta::NList<10, 11, 5, 3, 0x1e>>; // 1e = temp
//using mcp0P = Mcp9700aProvider<adcController, 0>;
//using mcp1P = Mcp9700aProvider<adcController, 1>;

//using iTempP = InternalTempProvider<adcController, 4>;

//using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
//                          Meta::List<mcp0P, mcp1P, iTempP>, 
//                          systemTimer, ibt>;

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;


using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;

using rxPin = q0Pin;
using txPin = void;
using swuart = External::SoftSerial::Usart<Meta::List<rxPin, void>, Component::Tca<0>,
                                            External::GPS::GpsProtocollAdapter<0, External::GPS::VTG>,
//                                            External::Hal::NullProtocollAdapter, 
                                            AVR::BaudRate<9600>>;
using isrRegistrar = IsrRegistrar<typename swuart::StartBitHandler, typename swuart::BitHandler>;

int main() {
#if 1
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

    terminalDevice::init<AVR::BaudRate<9600>>();
    
    swuart::init();
    
    evrouter::init();
    portmux::init();
    systemTimer::init();
//    adcController::init();
    
//    rpm0::init();   
//    rpm1::init();   
    
//    ibus::init();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    etl::StringBuffer<External::GPS::Sentence::DecimalMaxWidth> buffer;
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        while(true) {
            terminalDevice::periodic();
//            ibus::periodic();
//            adcController::periodic();
            
            systemTimer::periodic([&]{
                wdt::reset();
//                ibus::ratePeriodic();
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        External::GPS::VTG::speedRaw(buffer);
                        etl::outl<terminal>("* : "_pgm, buffer);
//                        rpm0::reset();
//                        rpm1::reset();
                    }
                });
            });
        }
    }
#endif
}

ISR(PORTA_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<rxPin::name_type>>();
}

ISR(TCA0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tca<0>::Ovf>();
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
