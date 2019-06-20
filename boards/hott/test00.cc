#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>

#include <external/hal/alarmtimer.h>
//#include <external/hott/sumdprotocoll.h>
#include <external/hott/sumdprotocolladapter.h>

#include <std/chrono>

#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortF = Port<F>;
using led = Pin<PortF, 5>; 
using dbg1 = Pin<PortF, 4>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using terminalDevice = Usart<Component::Usart<0>, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using rcUsart = AVR::Usart<AVR::Component::Usart<1>, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

namespace  {
    constexpr auto dt = 10_ms;
    constexpr auto fRtc = 128_Hz;
}

//using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using systemTimer = SystemTimer<Component::Timer<0, A>, dt>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    terminalDevice::init<BaudRate<9600>>();
    rcUsart::init<BaudRate<115200>>();
    
    systemTimer::init();
    
    led::template dir<Output>();     
    dbg1::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        terminalDevice::periodic();
        rcUsart::periodic();
        
        systemTimer::periodic([&]{
            dbg1::toggle();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    etl::outl<terminal>("test00"_pgm);
                }
                
            });
        });
    }
}

