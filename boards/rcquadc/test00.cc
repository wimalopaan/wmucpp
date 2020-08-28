#define NDEBUG

#define USE_IBUS
#define USE_DAISY

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/ibus/ibus.h>

#include <external/solutions/button.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto dt = 2_ms;
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SBUS
    constexpr auto fRtc = 1000_Hz;
#endif
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_PPM
    constexpr auto fRtc = 128_Hz;
#endif
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo / DBG
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Sensor

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using pwm = PWM::DynamicPwm8Bit<tcaPosition>;

using portmux = Portmux::StaticMapper<Meta::List<usart1Position, usart2Position>>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<servo>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using ledPin  = Pin<Port<D>, 2>; 

using pwmPin1 = Pin<Port<A>, 0>; 
using pwmPin2 = Pin<Port<A>, 1>; 
using pwmPin3 = Pin<Port<A>, 2>; 
using pwmPin4 = Pin<Port<A>, 3>; 

using end1Pin = Pin<Port<F>, 5>; 
using end2Pin = Pin<Port<D>, 1>; 
using end3Pin = Pin<Port<A>, 4>; 
using end4Pin = Pin<Port<A>, 5>; 

using inA1Pin = Pin<Port<F>, 3>; 
using inB1Pin = Pin<Port<F>, 1>; 
using inA2Pin = Pin<Port<D>, 3>; 
using inB2Pin = Pin<Port<D>, 6>; 
using inA3Pin = Pin<Port<C>, 3>; 
using inB3Pin = Pin<Port<C>, 2>; 
using inA4Pin = Pin<Port<A>, 6>; 
using inB4Pin = Pin<Port<A>, 7>; 

using dgbPin = Pin<Port<C>, 0>; // tx 
using ibusPin = Pin<Port<C>, 1>; // rx 

using csf1Pin = Pin<Port<F>, 2>; // ADC 
using csf2Pin = Pin<Port<D>, 7>; // ADC 
using csf3Pin = Pin<Port<D>, 5>; // ADC 
using csf4Pin = Pin<Port<D>, 0>; // ADC 

using sensorUartPin = Pin<Port<F>, 0>; // tx 

using daisyChain= Pin<Port<F>, 4>; 

#ifdef USE_DAISY
struct IBusThrough {
    inline static void init() {
        daisyChain::template dir<Output>();
    }
    inline static void on() {
        daisyChain::on();
    }
    inline static void off() {
        daisyChain::off();
    }
};
using ibt = IBusThrough;
#else
using ibt = void;
#endif

using ibus_sensor = IBus::Sensor<usart2Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<>, systemTimer, ibt
//                          , etl::NamedFlag<true>
//                           , etl::NamedFlag<true>
                          >;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 5, 7, 12, 0x1e>>; // 1e = temp

using adi_t = adcController::index_type;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    servo::init<BaudRate<115200>>();
    ibus_sensor::init();
    systemTimer::init();
    adcController::init();
    pwm::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
    etl::outl<terminal>("rcboard test00"_pgm);

    while(true) {
        adcController::periodic();
        servo::periodic();
        systemTimer::periodic([&]{
            ibus_sensor::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("c: "_pgm);
                }
            });
        });
    }
}

