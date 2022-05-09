#pragma once

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <mcu/internals/syscfg.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/rc/busscan.h>


namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units::literals;
    constexpr auto fRtc = 1000_Hz;
}


template<auto HWRev = 0, typename MCU = DefaultMcuType>
struct Devices;

// HW 22
template<typename MCU>
struct Devices<0, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;

#if defined(USE_TERMINAL_NO_LED) || defined(USE_SBUS_NO_PPM)
    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Alt1>; 
#endif
    
#ifndef USE_TERMINAL_NO_LED
    using ledPin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
    using led = AVR::ActiveHigh<ledPin, AVR::Output>;
    using blinkLed = External::Blinker2<led, systemTimer, 200_ms, 2000_ms>;
#endif

    using efPin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
    using ef    = AVR::ActiveLow<efPin, AVR::Input>;
    
    using ccl0Position = AVR::Portmux::Position<AVR::Component::Ccl<0>, AVR::Portmux::Default>;
    using lut0 = AVR::Ccl::SimpleLut<0, AVR::Ccl::Input::Tca0<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask>;
    using ccl1Position = AVR::Portmux::Position<AVR::Component::Ccl<1>, AVR::Portmux::Default>;
    using lut1 = AVR::Ccl::SimpleLut<1, AVR::Ccl::Input::Tca0<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask>;
    
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Alt1>;
    
    using pwm = AVR::PWM::DynamicPwm<tcaPosition>;
    
    using sppmPosition = AVR::Portmux::Position<AVR::Component::Tcb<0>, AVR::Portmux::Default>;
    using ppmIn =  AVR::Pin<AVR::Port<AVR::A>, 2>;
    using evch0 = AVR::Event::Channel<0, AVR::Event::Generators::Pin<ppmIn>>; 
    using ppm_user = AVR::Event::Route<evch0, AVR::Event::Users::Tcb<0>>;
    using ppmDevPosition = AVR::Component::Tcb<0>;

    using allRoutes = AVR::Event::Routes<ppm_user>;
    using evrouter = AVR::Event::Router<AVR::Event::Channels<evch0>, allRoutes>;
    
#if defined(USE_TERMINAL_NO_LED) || defined(USE_SBUS_NO_PPM)
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl0Position, ccl1Position, tcaPosition, usart0Position, sppmPosition>>;
#else
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl0Position, ccl1Position, tcaPosition, sppmPosition>>;
#endif
    
#ifdef USE_SBUS_NO_PPM
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
# ifndef USE_TERMINAL_NO_LED
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
# else
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
#endif
#else
    using sppm_input = External::Ppm::SinglePpmIn<sppmPosition::component_type>;
    using servo = External::Ppm::Adapter<sppm_input>;
    using servo_pa = servo::protocoll_adapter_type;
#endif
    
#ifdef USE_TERMINAL_NO_LED    
# ifdef USE_SBUS_NO_PPM
    using term_dev = servo;
# else
    using term_dev = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<64>>;
# endif
using terminal = etl::basic_ostream<term_dev>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    static inline void init() {
        portmux::init();
        evrouter::init();
        ccp::unlock([]{
            if constexpr(AVR::Concepts::At01Series<MCU>) {
                clock::template prescale<1>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
    }
    static inline void periodic() {
    }
};

