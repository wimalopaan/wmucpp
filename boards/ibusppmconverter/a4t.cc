//#define USE_IBUS
#define USE_SPORT
//#define USE_HOTT

#include "board.h"
#include "temp.h"

using adcController = External::Hal::AdcController<adc, Meta::NList<10, 11, 5, 3, 0x1e>>; // 1e = temp

#ifdef USE_SPORT
using rxPin = Pin<Port<A>, 1>; // alt1
using txPin = rxPin;
template<typename PA>
using sensorUsart = External::SoftSerial::Usart<Meta::List<rxPin, txPin>, Component::Tcd<0>, 
PA, AVR::BaudRate<57600>,
AVR::ReceiveQueueLength<0>,
AVR::SendQueueLength<64>,
etl::NamedFlag<true>
>;

using a1 = External::AnalogSensor<adcController, 0, std::ratio<500,1000>, std::ratio<1,100>, std::ratio<1,1>>;
using a2 = External::AnalogSensor<adcController, 1, std::ratio<500,1000>, std::ratio<1,100>, std::ratio<1,1>>;
using a3 = External::AnalogSensor<adcController, 2, std::ratio<500,1000>, std::ratio<1,100>, std::ratio<1,1>>;
using a4 = External::AnalogSensor<adcController, 3, std::ratio<500,1000>, std::ratio<1,100>, std::ratio<1,1>>;

template<typename Sensor>
struct TempProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Temp1;
    inline static uint32_t value() {
        return Sensor::value();
    }
};

using p1 = TempProvider<a1>;
using p2 = TempProvider<a2>;
using p3 = TempProvider<a3>;
using p4 = TempProvider<a4>;

using sport = External::SPort::Sensor<External::SPort::SensorId::ID3, sensorUsart, systemTimer, 
                                       Meta::List<p1, p2, p3, p4>>;

using isrRegistrar = IsrRegistrar<sport::uart::StartBitHandler, sport::uart::BitHandler>;

using portmux = Portmux::StaticMapper<Meta::List<>>;
#endif

#ifdef USE_IBUS
using mcp0P = Mcp9700aProvider<adcController, 0>;
using mcp1P = Mcp9700aProvider<adcController, 1>;
using mcp2P = Mcp9700aProvider<adcController, 2>;
using mcp3P = Mcp9700aProvider<adcController, 3>;

using iTempP = InternalTempProvider<adcController, 4>;

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<mcp0P, mcp1P, mcp2P, mcp3P>, 
                          systemTimer, ibt>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;
#endif

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
    
#ifdef USE_IBUS
    ibus::init();
#endif
    
#ifdef USE_SPORT
    sport::init();
#endif
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    {
#ifdef USE_SPORT
        etl::Scoped<etl::EnableInterrupt<>> ei;        
#endif
        while(true) {
    #ifdef USE_IBUS
            ibus::periodic();
    #endif
    #ifdef USE_SPORT
            sport::periodic();
    #endif
            adcController::periodic();
            
            systemTimer::periodic([&]{
                wdt::reset();
    #ifdef USE_IBUS
                ibus::ratePeriodic();
    #endif
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                    }
                });
            });
        }
    }
}

#ifdef USE_SPORT
ISR(PORTA_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<rxPin::name_type>>();
}

ISR(TCD0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tcd<0>::Ovf>();
}
#endif

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
