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

using tcbPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart3Position, tcaPosition, tcbPosition>>;

using evch0 = Event::Channel<0, Event::Generators::PitDiv<1024>>;
using evch1 = Event::Channel<1, Event::Generators::PitDiv<512>>;
using evuser0 = Event::Route<evch1, Event::Users::Lut<0, A>>;
using evuser1 = Event::Route<evch0, Event::Users::EvOut<A>>;
using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Event<A>, Ccl::Input::Mask, Ccl::Input::Mask>;

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

int main() {
    evrouter::init();

    lut0::init(std::byte{0x55});
    
    using rtc_t = DefaultMcuType::Rtc;
    static constexpr auto mcu_rtc = getBaseAddr<rtc_t>;
    
    while(mcu_rtc()->pitstatus.isSet<rtc_t::PitStatus_t::ctrlbusy>());
    
//    mcu_rtc()->pitctrla.template set<rtc_t::pitPrescalerValues[8].bits>();

    while(mcu_rtc()->pitstatus.isSet<rtc_t::PitStatus_t::ctrlbusy>());

    mcu_rtc()->pitctrla.template add<rtc_t::PitCtrlA_t::enable>();
    
    
}

