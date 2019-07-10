#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>

#include <external/hal/alarmtimer.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>

#include <std/chrono>

#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using dbg1 = Pin<PortA, 5>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;

using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

namespace  {
    constexpr auto dt = 2000_us;
    constexpr auto fRtc = 500_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, systemTimer>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

int main() {
    dbg1::template dir<Output>();
    portmux::init();

    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();

    sensor::init();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        sensor::periodic();
        
        systemTimer::periodic([&]{
            dbg1::toggle();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("test00"_pgm);
                }
                
            });
        });
    }
}

