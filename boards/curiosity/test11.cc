#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/event.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/pit.h>
#include <mcu/internals/adc.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/series01/sppm_in.h>

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
using PortD = Port<D>;
using PortF = Port<F>;
using PortE = Port<E>;

using led = Pin<PortF, 5>; 
using pf2 = Pin<PortF, 2>; 
using pb1 = Pin<PortB, 1>; 
//using pa2 = Pin<PortA, 2>; 
using pe3 = Pin<PortE, 3>; // ppm (s.u.)
using rpmPin = Pin<PortE, 0>; 

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

using tcbPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;

using ccl0Position = Portmux::Position<Component::Ccl<0>, Portmux::Default>;
using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart3Position, tcaPosition, tcbPosition, ccl0Position, ccl1Position>>;
//using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart3Position>>;
//using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position>>;

using evch0 = Event::Channel<0, Event::Generators::PitDiv<1024>>;
using evch1 = Event::Channel<4, Event::Generators::Pin<pe3>>; 
using evuser0 = Event::Route<evch0, Event::Users::Lut<0, A>>;
using evuser1 = Event::Route<evch0, Event::Users::Lut<1, A>>;
using evuser2 = Event::Route<evch1, Event::Users::Tcb<0>>;
using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1, evuser2>>;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Tca0<0>, Ccl::Input::Event<A>, Ccl::Input::Mask>;
using lut1 = Ccl::SimpleLut<1, Ccl::Input::Tca0<0>, Ccl::Input::Event<A>, Ccl::Input::Mask>;

using pit = Rtc::Pit<>;

using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; // in andere header Datei verschieben

namespace  {
//    constexpr auto dt = 2_ms;
    constexpr auto dt = 2000_us;
    constexpr auto fRtc = 500_Hz;
    constexpr auto fPwm = 1000_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
//using systemTimer = SystemTimer<Component::Timer<0, A>, dt>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using sensor = Hott::Experimental::Sensor<usart3Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, systemTimer>;

using rpm = External::Rpm::RpmGpio<rpmPin, systemTimer>;

using isrRegistrar = IsrRegistrar<rpm::ImpulsIsr>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1>>;
using pd4 = Pin<PortD, 0>; 

int main() {
    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    evrouter::init();
   
    lut0::init(std::byte{0x08});
    lut1::init(std::byte{0x02});

    pit::init();
    
    terminalDevice::init<BaudRate<9600>>();
    rcUsart::init<BaudRate<115200>>();
    sensor::init();
    
    systemTimer::init();
    
    led::template dir<Output>();     
//    pf2::template dir<Output>();     
    pb1::template dir<Output>();     
//    pa2::template dir<Output>();     
    
//    pe3::template dir<Input>();     

//    pa2::on();
    pb1::on();
    
    pwm::init();
    pwm::frequency(fPwm);
//    pwm::on<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>();
////    pwm::off<PWM::WO<2>>();

    pwm::duty<PWM::WO<0>>(1000);

    ppm::init();
    
    rpm::init();

    adcController::init();
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        
        const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
        while(true) {
            terminalDevice::periodic();
            rcUsart::periodic();
            sensor::periodic();
            adcController::periodic();
            systemTimer::periodic([&]{
                pf2::toggle(); // 5,8us - 17us
                sensor::ratePeriodic();

                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        led::toggle();
                        etl::outl<terminal>("test11: "_pgm, ++counter, " ch0: "_pgm, sumd::value(0).toInt());
//                        etl::outl<terminal>("co: "_pgm, sensor::collisions(), " ar: "_pgm, sensor::asciiPackages(), " br: "_pgm, sensor::binaryPackages());
                        etl::outl<terminal>("ppm: "_pgm, ppm::value());
                        etl::outl<terminal>("adc0: "_pgm, adcController::value(0).toInt());
                        etl::outl<terminal>("rpm: "_pgm, rpm::diff());
                        if (auto c = terminalDevice::get()) {
                            etl::outl<terminal>("c: "_pgm, *c);
                        }
                    }
                });
            });
        }
    }
}

ISR(PORTE_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<E>>();
}
