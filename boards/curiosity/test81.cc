#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>

#include <external/hal/alarmtimer.h>
#include <external/sbus/sbus.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortF = Port<F>;

using led = Pin<PortF, 5>; 
using pa0 = Pin<PortF, 2>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position>>;

using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using rm = Usart<usart1Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;

namespace  {
    constexpr auto fRtc = 71_Hz; // 14ms
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

int main() {
    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    terminalDevice::init<BaudRate<9600>>();
    rm::init<BaudRate<115200>>(); 
   
    systemTimer::init();
    
    led::template dir<Output>();     
    pa0::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        terminalDevice::periodic();
        rm::periodic();
        
        systemTimer::periodic([&]{
            pa0::toggle();
                       
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    etl::outl<terminal>("cnt: "_pgm, counter++);
                    rm::put(std::byte{'$'});
                    rm::put(std::byte{'s'});
                    rm::put(std::byte{'9'});
                    rm::put(std::byte{'1'});
                    rm::put(std::byte{':'});
                    rm::put(std::byte{'4'});
                    rm::put(std::byte{'2'});
                    rm::put(std::byte{'\n'});
                }
            });
        });
    }
}

