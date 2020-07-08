#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/sbus/sbus.h>
#include <external/bluetooth/roboremo.h>

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

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart2Position, usart1Position>>;

using terminalDevice = Usart<usart2Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using sbus = External::SBus::Output::Generator<usart1Position>;

using robo_pa = External::RoboRemo::ProtocollAdapter<0>;
using robo = Usart<usart0Position, robo_pa, UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

namespace  {
    constexpr auto fRtc = 71_Hz; // 14ms
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 0x1e>>; // 1e = temp


int main() {
    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    
    terminalDevice::init<BaudRate<9600>>();
    robo::init<BaudRate<9600>>();
    
    sbus::init(); // even parity, 2 stop bits
   
    systemTimer::init();
    
    led::template dir<Output>();     
    pa0::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    adcController::init();
    
    using adi_t = adcController::index_type;
    
    while(true) {
        adcController::periodic();
        terminalDevice::periodic();
        robo::periodic();
        sbus::periodic();
        
        systemTimer::periodic([&]{
            pa0::toggle();
            
            sbus::ratePeriodic(); // 14ms
            
            const uint16_t v0 = adcController::value(adi_t{0}).toInt();
            sbus::output[0] = (v0 - 512) + 992;
            
            const uint16_t v1 = robo_pa::propValues[0];
            sbus::output[1] = v1 + 992;
            
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    etl::outl<terminal>("cnt: "_pgm, counter++, " v0: "_pgm, v0, " v1: "_pgm, v1);
                }
            });
        });
    }
}

