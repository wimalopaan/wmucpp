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
#include <mcu/internals/eeprom.h>

#include <external/solutions/tick.h>

#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

struct ApplData : public EEProm::DataBase<ApplData> {
    uint16_t mSpeed;
    uint16_t mBalance;
    uint16_t mCurrent;
#ifndef NO_SHIFT
    uint16_t mShift;
#endif
    uint16_t mSine;
    uint8_t mMagic;
};

template<typename MCU>
struct Devices<Board1614_01, MCU> {
    using PortA = AVR::Port<AVR::A>;
    using PortB = AVR::Port<AVR::B>;

    using led = AVR::Pin<PortB, 2>; 
    
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;
        
    using eeprom = EEProm::Controller<ApplData>;
    
    using lut0 = AVR::Ccl::SimpleLut<0, AVR::Ccl::Input::Tca0<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask>;
    using lut1 = AVR::Ccl::SimpleLut<1, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Tca0<1>, AVR::Ccl::Input::Mask>;
    
    using pwm = AVR::PWM::DynamicPwm<tcaPosition>;
    using stepper = External::StepMotor::MP6513Driver<pwm, lut0, lut1>; 
    
    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Alt1>; // Servo: Fehler auf Platine: RX
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<tcaPosition, usart0Position>>;
    
//    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
//    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
//    using term_dev = servo;
    
    using scanDevPosition = usart0Position;
    using scan_term_dev = void;
    using ppmDevPosition = void;
    using evrouter = void;
    using scanLedPin = AVR::ActiveHigh<led, AVR::Output>;
    
    static inline void init() {
        portmux::init();
        ccp::unlock([]{
            clock::prescale<1>();
        });
        systemTimer::init();
    }

    static inline void periodic() {
    }
};

template<typename MCU>
struct Devices<NoBoard1614, MCU> {
    using PortA = AVR::Port<AVR::A>;
    using PortB = AVR::Port<AVR::B>;
    
    using an0 = AVR::Pin<PortA, 1>; 
    using an1 = AVR::Pin<PortA, 2>; 
    using gra = AVR::PinGroup<Meta::List<an0, an1>>;
    
    using bn0 = AVR::Pin<PortA, 5>; 
    using bn1 = AVR::Pin<PortA, 6>; 
    using grb = AVR::PinGroup<Meta::List<bn0, bn1>>;
    
    using ppmIn = AVR::Pin<PortB, 2>;
    
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;
    
    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<10>, AVR::Vref::V4_3>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<3>>;
    using adc_i_t = adcController::index_type;
    
    using lut0 = AVR::Ccl::SimpleLut<0, AVR::Ccl::Input::Tca0<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask>;
    using lut1 = AVR::Ccl::SimpleLut<1, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Tca0<1>, AVR::Ccl::Input::Mask>;
    
    using pwm = AVR::PWM::DynamicPwm<tcaPosition>;
    using stepper = External::StepMotor::DirPhaseDriver<pwm, gra, grb, etl::NamedFlag<true>, lut0, lut1>; // stepper direct on pins
    //using stepper = Stepper<pwm, gra, grb, etl::NamedFlag<false>, lut0, lut1>; // using drv8835
    
    using sppmPosition = AVR::Portmux::Position<AVR::Component::Tcb<0>, AVR::Portmux::Default>;
    using evch1 = AVR::Event::Channel<1, AVR::Event::Generators::Pin<ppmIn>>; 
    using ppm_user = AVR::Event::Route<evch1, AVR::Event::Users::Tcb<0>>;
    using sppm_input = External::Ppm::SinglePpmIn<sppmPosition::component_type>;
    
    using evrouter = AVR::Event::Router<AVR::Event::Channels<evch1>, AVR::Event::Routes<ppm_user>>;
    using portmux = AVR::Portmux::StaticMapper<Meta::List<tcaPosition, sppmPosition>>;

    using eeprom = EEProm::Controller<ApplData>;
    
    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; // Sensor
    //using term_dev = Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
    //using term_dev = void;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    using term_dev = servo;
};
