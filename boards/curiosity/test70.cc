#define NDEBUG

#define USE_SBUS
//#define USE_IBUS

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>

#include <external/hal/alarmtimer.h>
#include <external/sbus/sbus.h>
#include <external/ibus/ibus.h>

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

#ifdef USE_SBUS
using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
#endif
#ifdef USE_IBUS
using servo_pa = IBus::Servo::ProtocollAdapter<0>;
#endif

using servo = AVR::Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;


int main() {
//    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    terminalDevice::init<BaudRate<9600>>();

#ifdef USE_SBUS
    servo::init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
//    servo::txEnable<false>();
#endif
#ifdef USE_IBUS
    servo::init<AVR::BaudRate<115200>>();
//    servo::txEnable<false>();
#endif
    systemTimer::init();
    
    led::template dir<Output>();     
    pa0::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

    using ch_t = servo_pa::channel_t;
    
    while(true) {
        terminalDevice::periodic();
        servo::periodic();        
        
        systemTimer::periodic([&]{
#ifdef USE_SBUS
            servo_pa::ratePeriodic();
#endif
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    etl::outl<terminal>(" ch9: "_pgm, servo_pa::value(ch_t{9}).toInt());
//                    etl::outl<terminal>(" ch5: "_pgm, servo_pa::value(ch_t{5}).toInt());
//                    etl::outl<terminal>(" ch15: "_pgm, servo_pa::value(ch_t{14}).toInt());
//                    etl::outl<terminal>(" ch16: "_pgm, servo_pa::value(ch_t{15}).toInt());
//                    etl::outl<terminal>(" ch0: "_pgm, servo_pa::value(ch_t{0}).toInt());
//                    etl::outl<terminal>(" ch5: "_pgm, servo_pa::value(ch_t{5}).toInt());
                }
            });
        });
    }
}

