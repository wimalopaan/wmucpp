#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>

#include <external/hal/alarmtimer.h>
#include <external/hott/sumdprotocolladapter.h>

#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortB = Port<B>;
using PortF = Port<F>;

using led = Pin<PortF, 5>; 
using pf2 = Pin<PortF, 2>; 
using pb1 = Pin<PortB, 1>; 
using pa2 = Pin<PortA, 2>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart3Position = Portmux::Position<Component::Usart<3>, Portmux::Default>;


using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using rcUsart = AVR::Usart<usart1Position, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltD>;
using pwm = PWM::DynamicPwm<tcaPosition>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart3Position, tcaPosition>>;

namespace  {
//    constexpr auto dt = 2_ms;
    constexpr auto dt = 2000_us;
    constexpr auto fRtc = 512_Hz;
    constexpr auto fPwm = 1000_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
//using systemTimer = SystemTimer<Component::Timer<0, A>, dt>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using sensor = Hott::Experimental::Sensor<usart3Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, systemTimer>;

int main() {
    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    terminalDevice::init<BaudRate<9600>>();
    rcUsart::init<BaudRate<115200>>();
   
    sensor::init();
    
    systemTimer::init();
    
    led::template dir<Output>();     
    pf2::template dir<Output>();     
    pb1::template dir<Output>();     
    pa2::template dir<Output>();     

    pa2::on();
    
    pwm::init();
    pwm::frequency(fPwm);
    pwm::on<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>();
//    pwm::off<PWM::WO<2>>();

    pwm::duty<PWM::WO<0>>(1000);
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        terminalDevice::periodic();
        rcUsart::periodic();
        sensor::periodic();
        
        pb1::toggle();
        
        systemTimer::periodic([&]{
            pf2::toggle(); // 5,8us - 17us
            sensor::ratePeriodic();
            
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    etl::outl<terminal>("test04: "_pgm, ++counter, " ch0: "_pgm, sumd::value(0).toInt());
                    etl::outl<terminal>("co: "_pgm, sensor::collisions(), " ar: "_pgm, sensor::asciiPackages(), " br: "_pgm, sensor::binaryPackages());
                    if (auto c = terminalDevice::get()) {
                        etl::outl<terminal>("c: "_pgm, *c);
                    }
                }
            });
        });
    }
}

