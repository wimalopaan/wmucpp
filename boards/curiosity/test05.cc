#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>
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
using pa0 = Pin<PortA, 0>; 
using pa3 = Pin<PortA, 3>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart3Position = Portmux::Position<Component::Usart<3>, Portmux::Default>;

using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using rcUsart = AVR::Usart<usart1Position, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
using pwm = PWM::DynamicPwm<tcaPosition>;

using cclPosition = Portmux::Position<Component::Ccl<0>, Portmux::Default>;

//using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart3Position, tcaPosition, cclPosition>>;
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

using sensor = Hott::Experimental::Sensor<usart3Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, systemTimer>;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    portmux::init();
    
    pwm::init();
    pwm::on<PWM::WO<0>, PWM::WO<1>>();
    pwm::frequency(fPwm);
    pwm::duty<PWM::WO<0>>(1000);
    
    
    using ccl_t = DefaultMcuType::Ccl;
    static constexpr auto mcu_ccl = getBaseAddr<ccl_t>;
    
    mcu_ccl()->lut0ctrlb.add<ccl_t::Lut0CtrlB_t::in0_tca0>();
    *mcu_ccl()->truth0 = 0x55_B;
    
    mcu_ccl()->lut0ctrla.add<ccl_t::Lut0CtrlA_t::enable | ccl_t::Lut0CtrlA_t::outenable>();
    
    mcu_ccl()->ctrla.add<ccl_t::CtrlA_t::enable>();
    
    
    //    pwm::set<PWM::WO<0>>();
    
    //    pwm::off<PWM::WO<2>>();
    //    pwm::reset<PWM::WO<0>>();
}

