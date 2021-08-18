#pragma once

#include <cmath>

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/event.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>

#include <external/hal/adccontroller.h>
#include <external/solutions/tick.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

struct ApplData {
};

template<typename MCU>
struct Devices<Board412_01, MCU> {
    using PortA = AVR::Port<AVR::A>;
    
    using an0 = AVR::Pin<PortA, 6>; 
    using bn0 = AVR::Pin<PortA, 7>; 
    
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;
    
    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<10>, AVR::Vref::V4_3>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<2>>;
    using adc_i_t = adcController::index_type;
    
    using pwm = AVR::PWM::DynamicPwm<tcaPosition>;
    using stepper = External::StepMotor::DirPhaseDriver<pwm, an0, bn0, etl::NamedFlag<true>, void, void>; // stepper direct on pins

    using term_dev = void;

    using eeprom = EEProm::Controller<ApplData>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<tcaPosition>>;
    
};


template<typename MCU>
struct Devices<NoBoard412, MCU> {
    using PortA = AVR::Port<AVR::A>;
    
    using an0 = AVR::Pin<PortA, 2>; 
    using gra = AVR::PinGroup<Meta::List<an0>>;
    
    using bn0 = AVR::Pin<PortA, 6>; 
    using grb = AVR::PinGroup<Meta::List<bn0>>;
    
    using ppmIn = AVR::Pin<PortA, 7>;
    
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;

    using eeprom = EEProm::Controller<ApplData>;
    
    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<10>, AVR::Vref::V4_3>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<7>>;
    using adc_i_t = adcController::index_type;
    
    using pwm = AVR::PWM::DynamicPwm<tcaPosition>;
    using stepper = External::StepMotor::DirPhaseDriver<pwm, gra, grb, etl::NamedFlag<true>, void, void>; // stepper direct on pins
    
    using sppmPosition = AVR::Portmux::Position<AVR::Component::Tcb<0>, AVR::Portmux::Default>;
    using evch1 = AVR::Event::Channel<0, AVR::Event::Generators::Pin<ppmIn>>; 
    using ppm_user = AVR::Event::Route<evch1, AVR::Event::Users::Tcb<0>>;
    using sppm_input = External::Ppm::SinglePpmIn<sppmPosition::component_type>;
    
    using evrouter = AVR::Event::Router<AVR::Event::Channels<evch1>, AVR::Event::Routes<ppm_user>>;
    using portmux = AVR::Portmux::StaticMapper<Meta::List<tcaPosition, sppmPosition>>;
    
    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; // Sensor
    using term_dev = void;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
};
