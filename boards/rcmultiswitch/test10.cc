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
#include <external/ibus/ibus.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using led7 = Pin<Port<B>, 3>; 
using led6 = Pin<Port<A>, 7>; 
using led5 = Pin<Port<A>, 5>; 
using led4 = Pin<Port<A>, 4>; 
using led3 = Pin<Port<A>, 3>; 
using led2 = Pin<Port<B>, 2>; 
using led1 = Pin<Port<B>, 1>; 
using led0 = Pin<Port<B>, 0>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;

using ibus_switch = IBus::Switch::Switch2<servo_pa>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
using terminal = etl::basic_ostream<servo>;

namespace  {
    constexpr auto dt = 2_ms;
    constexpr auto fRtc = 128_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;


int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    servo::init<BaudRate<115200>>();
    
    systemTimer::init();
    
    led0::template dir<Output>();     
    led1::template dir<Output>();     
    led2::template dir<Output>();     
    led3::template dir<Output>();     
    led4::template dir<Output>();     
    led5::template dir<Output>();     
    led6::template dir<Output>();     
    led7::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

    etl::outl<terminal>("test10"_pgm);

    uint16_t counter{};
    while(true) {
        servo::periodic();
        
        systemTimer::periodic([&]{
            ibus_switch::periodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("c: "_pgm, counter++);
                }
            });
        });
    }
}

