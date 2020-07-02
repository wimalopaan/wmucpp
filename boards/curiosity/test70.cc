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
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position>>;

using terminalDevice = Usart<usart2Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

namespace  {
    constexpr auto fRtc = 1000_Hz; 
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
using servo = AVR::Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;


int main() {
    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    terminalDevice::init<BaudRate<9600>>();
   
    servo::init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
//    servo::txEnable<false>();
    
    systemTimer::init();
    
    led::template dir<Output>();     
    pa0::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

    using ch_t = servo_pa::channel_t;
    
    while(true) {
        terminalDevice::periodic();
        servo::periodic();        
        
        systemTimer::periodic([&]{
            pa0::toggle();
            
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    etl::outl<terminal>("cnt: "_pgm, servo_pa::c, " ch0: "_pgm, servo_pa::value(ch_t{9}).toInt());
                }
            });
        });
    }
}

