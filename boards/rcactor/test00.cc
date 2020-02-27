#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>

#include <external/hal/alarmtimer.h>
#include <external/hott/sumdprotocolladapter.h>
//#include <external/hott/sumdprotocoll.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortF = Port<F>;

using led = Pin<PortF, 5>; 
using pf2 = Pin<PortF, 2>; 
using pa0 = Pin<PortA, 0>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

//using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
//using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Alt1>;

//using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltB>;

//using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, tcaPosition>>;

//using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
//using terminal = etl::basic_ostream<terminalDevice>;

//using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
//using rcUsart = AVR::Usart<usart1Position, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

namespace  {
    constexpr auto dt = 2_ms;
    constexpr auto fRtc = 128_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
//using systemTimer = SystemTimer<Component::Timer<0, A>, dt>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

int main() {
//    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
//    terminalDevice::init<BaudRate<9600>>();
////    rcUsart::init<BaudRate<115200>>();
    
    
    systemTimer::init();
    
    led::template dir<Output>();     
    pf2::template dir<Output>();     
    pa0::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(10_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        pf2::toggle();
//        terminalDevice::periodic();
//        rcUsart::periodic();
        
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    pa0::toggle();
//                    etl::outl<terminal>("test00"_pgm);
                }
                
            });
        });
    }
}

