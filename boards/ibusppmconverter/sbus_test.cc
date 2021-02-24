#define NDEBUG

#define USE_SBUS

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

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

namespace  {
    constexpr auto fRtc = 1000_Hz; 
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;

using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
using terminal = etl::basic_ostream<servo>;

int main() {
//    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    servo::init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
//    servo::txEnable<false>();
    servo::rxInvert(true);
    
    systemTimer::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

    using ch_t = servo_pa::channel_t;
    
    while(true) {
        servo::periodic();        
        
        systemTimer::periodic([&]{
            servo_pa::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
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

