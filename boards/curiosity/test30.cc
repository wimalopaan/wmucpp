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
#include <mcu/internals/sleep.h>
#include <mcu/internals/sigrow.h>
#include <mcu/pgm/pgmarray.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>
#include <external/ibus/ibus.h>
#include <external/units/music.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>

#include <external/solutions/debug/logic_analyzer.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
    constexpr auto fRtc = 2048_Hz;
}

using systemTimer = SystemTimer<Component::Pit<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using ledPin    = ActiveHigh<Pin<Port<F>, 5>, Output>;
using assertPin = ActiveHigh<Pin<Port<F>, 6>, Output>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using sleep = Sleep<>;

struct VoltageProvider {
};
struct CurrentProvider {
};
struct IBusThrough {
    inline static constexpr void on() {}
    inline static constexpr void off() {}
};
using ibt = IBusThrough;

using dbg = PinGroup<Meta::List<Pin<Port<A>, 3>, Pin<Port<A>, 4>, Pin<Port<A>, 5>, Pin<Port<A>, 6>, Pin<Port<A>, 7>> >;
using tParallelDebug = External::Debug::TriggeredParallel<Pin<Port<A>, 2>, dbg>;

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<VoltageProvider, CurrentProvider>, systemTimer,
                          ibt, tParallelDebug>;

using terminalDevice = AVR::Usart<usart2Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

////using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
////using adcController = External::Hal::AdcController<adc, Meta::NList<1, 5, 0x1e>>; // 1e = temp
//using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t, std::ratio<121,21>>; // todo: richtiger scale faktor
//using currentConverter = Hott::Units::Converter<adc, Hott::Units::current_t, std::ratio<11117,1000>>; // todo: richtiger scale faktor

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, tcaPosition>>;

int main() {
    assertPin::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    ledPin::init();
    portmux::init();
    systemTimer::init();
//    adcController::init();
    
    ibus::init();
    terminalDevice::init<AVR::BaudRate<9600>>();
    
    sleep::template init<sleep::PowerDown>();
    
    const auto periodicTimer = alarmTimer::create(300_ms, External::Hal::AlarmFlags::Periodic);
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        etl::outl<terminal>("test30"_pgm);
        
        while(true) {
            ibus::periodic();
            terminalDevice::periodic();
            
//            adcController::periodic();
            
            systemTimer::periodic([&]{
                ibus::ratePeriodic();
                
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        ledPin::toggle();
                        etl::outl<terminal>("SP: "_pgm, uint8_t(ibus::pa::mState), " SR: "_pgm, uint8_t(ibus::responder::mReply), 
                                            " nB: "_pgm, ibus::nBytes, " nP: "_pgm, ibus::nPackets, " N1: "_pgm, ibus::mFirstSensorNumber.toInt(), 
                                            " N2: "_pgm, ibus::mLastSensorNumber.toInt());
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
