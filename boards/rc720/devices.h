#pragma once

#include <mcu/avr.h>

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
#include <external/hal/nulldev.h>

#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/rc/busscan.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units::literals;
    constexpr auto fRtc = 1000_Hz;
    
    template<typename Pin>
    struct IBusThrough {
        inline static void init() {
            Pin::template dir<AVR::Output>();
        }
        inline static void on() {
            Pin::on();
        }
        inline static void off() {
            Pin::off();
        }
    };
    
    
}

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, External::Bus::fRtc>;

    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Alt1>; // Servo / Debug
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; // Sensor
        
    using servoPosition = usart0Position; 
    using scanDevPosition = servoPosition;
    
    using sensorPosition = usart2Position; // Sensor
    using scanTermPosition = servoPosition;

#ifdef NDEBUG
    using scan_term_dev = void;
#else
    using scan_term_dev = AVR::Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#endif
    
#ifndef NDEBUG
//    using assertPin = Pin<Port<A>, 0>; 
#endif
    
    using daisyChain= AVR::Pin<AVR::Port<AVR::D>, 4>; 
    
    using led1 = AVR::Pin<AVR::Port<AVR::F>, 1>;
    using led2 = AVR::Pin<AVR::Port<AVR::C>, 2>;
    
    using scanLedPin = AVR::ActiveHigh<led1, AVR::Output>;
    
    // lut2 out:
    using ccl2Position = AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Alt1>;
    using lut2 = AVR::Ccl::SimpleLut<2, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2>, AVR::Ccl::Input::Mask>;
    
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
    
    using ibt = IBusThrough<daisyChain>;
    
    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<12>, AVR::Vref::V2_048>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<0, 19, 0x42>>; // 0x42 = temp
    using adc_i_t = adcController::index_type;
    
    using pwm3Position = AVR::Portmux::Position<AVR::Component::Tcb<0>, AVR::Portmux::Alt1>;
    using pwm4Position = AVR::Portmux::Position<AVR::Component::Tcb<1>, AVR::Portmux::Alt1>;
    using pwm5Position = AVR::Portmux::Position<AVR::Component::Tcb<2>, AVR::Portmux::Default>;
    
    using ppmDevPosition = void;

//    using evrouter = Event::Router<Event::Channels<>, Event::Routes<>>;
    using evrouter = External::Hal::NullDevice<>;
    
//    using eeprom = EEProm::Controller<Data>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl2Position, tcaPosition, servoPosition, sensorPosition>>;
    
    static inline void init() {
        portmux::init();
        if constexpr(!std::is_same_v<evrouter, void>) {
            evrouter::init();
        }
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDa32<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
            }
            else if constexpr(AVR::Concepts::At01Series<MCU>) {
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

