#pragma once

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

template<auto HWRev, typename MCU = DefaultMcuType>
struct Devices;

template<typename MCU>
struct Devices<0, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, External::Bus::fRtc>;
    
    using spiPosition = AVR::Portmux::Position<AVR::Component::Spi<0>, AVR::Portmux::Default>;
    
    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; // terminal
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Default>; // Servo (in / out)
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Alt1>; // Sensor
    
    using servoPosition = usart1Position; // servo / link / telem-mirror
    using scanDevPosition = servoPosition;
    
    using sensorPosition = usart2Position; // sensor / sbus-out

    using scanTermPosition = usart0Position;  // terminal
    using scan_term_dev = AVR::Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
    using terminal = etl::basic_ostream<scan_term_dev>;

    using dbgPin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
    
    using ssPin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
    
    using daisyChain= AVR::Pin<AVR::Port<AVR::F>, 5>; 
    
    using pwmInPin = AVR::Pin<AVR::Port<AVR::C>, 3>;
    
    using WPin = AVR::Pin<AVR::Port<AVR::D>, 0>;
    using IPin = WPin;
    using VPin = AVR::Pin<AVR::Port<AVR::D>, 1>;
    using BPin = VPin;
    using UPin = AVR::Pin<AVR::Port<AVR::D>, 2>;
    using APin = UPin;
    
    using ledPin = AVR::Pin<AVR::Port<AVR::D>, 6>;
    using led = AVR::ActiveHigh<ledPin, AVR::Output>;
    
    using enablePin = AVR::Pin<AVR::Port<AVR::D>, 7>;
    using enable = AVR::ActiveHigh<enablePin, AVR::Output>;
    
    using scanLedPin = AVR::ActiveHigh<ledPin, AVR::Output>;
    
    // lut3 out: pf3
    using ccl3Position = AVR::Portmux::Position<AVR::Component::Ccl<3>, AVR::Portmux::Default>;
    using lut3 = AVR::Ccl::SimpleLut<3, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2>>;
    
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltF>;
    using pwm = External::PWM::DynamicPwm<tcaPosition>;
    
//    using driver = Driver<pwm>;
    
    template<typename Command, typename Result>
    using spi = AVR::SpiSync<spiPosition, Command, Result, AVR::QueueLength<2>, ssPin>;
    
    using encoder = External::AS5147::Encoder<spi>;
    
//    using ibt = IBusThrough<daisyChain>;
    
    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<12>, AVR::Vref::V4_096>; // 3 = Strom, 4 = VBatt, 5 = Temp
    using adcController = External::Hal::AdcController<adc, Meta::NList<3, 4, 5, 0x42>>; // 0x42 = temp
    using adc_i_t = adcController::index_type;

    using ppmDevPosition = void;
    
    //    using evrouter = Event::Router<Event::Channels<>, Event::Routes<>>;
    using evrouter = void;
    
    //    using eeprom = EEProm::Controller<Data>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl3Position, tcaPosition, servoPosition, sensorPosition, scanTermPosition>>;
    
    static inline void init() {
        portmux::init();
        if constexpr(!std::is_same_v<evrouter, void>) {
            //            evrouter::init();
        }
        ccp::unlock([]{
            clock::template init<Project::Config::fMcuMhz>();
        });
        systemTimer::init(); 
    }
    static inline void periodic() {
    }
};

template<typename MCU>
struct Devices<1, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, External::Bus::fRtc>;
    
    using spiPosition = AVR::Portmux::Position<AVR::Component::Spi<0>, AVR::Portmux::Default>;
    
    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; // terminal
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Default>; // Servo (in / out)
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Alt1>; // Sensor
    
    using servoPosition = usart1Position; // servo / link / telem-mirror
    using scanDevPosition = servoPosition;
    
    using sensorPosition = usart2Position; // sensor / sbus-out

    using scanTermPosition = usart0Position;  // terminal
    using scan_term_dev = AVR::Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
    using terminal = etl::basic_ostream<scan_term_dev>;

    using dbgPin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
    
    using ssPin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
    
    using daisyChain= AVR::Pin<AVR::Port<AVR::F>, 5>; 
    
    using pwmInPin = AVR::Pin<AVR::Port<AVR::C>, 3>;
    
    using WPin = AVR::Pin<AVR::Port<AVR::D>, 0>;
    using IPin = WPin;
    using VPin = AVR::Pin<AVR::Port<AVR::D>, 1>;
    using BPin = VPin;
    using UPin = AVR::Pin<AVR::Port<AVR::D>, 2>;
    using APin = UPin;
    
    using ledPin = AVR::Pin<AVR::Port<AVR::D>, 6>;
    using led = AVR::ActiveHigh<ledPin, AVR::Output>;
    
    using enablePin = AVR::Pin<AVR::Port<AVR::D>, 7>;
    using enable = AVR::ActiveHigh<enablePin, AVR::Output>;
    
    using scanLedPin = AVR::ActiveHigh<ledPin, AVR::Output>;
    
    // lut3 out: pf3
    using ccl3Position = AVR::Portmux::Position<AVR::Component::Ccl<3>, AVR::Portmux::Default>;
    using lut3 = AVR::Ccl::SimpleLut<3, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2>>;
    
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltF>;
    using pwm = External::PWM::DynamicPwm<tcaPosition>;
    
//    using driver = Driver<pwm>;
    
    template<typename Command, typename Result>
    using spi = AVR::SpiSync<spiPosition, Command, Result, AVR::QueueLength<2>, ssPin>;
    
    using encoder = External::AS5147::Encoder<spi, etl::integral_constant<uint8_t, 11>>;
    
//    using ibt = IBusThrough<daisyChain>;
    
    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<12>, AVR::Vref::V4_096>; // 3 = Strom, 4 = VBatt, 5 = Temp
    using adcController = External::Hal::AdcController<adc, Meta::NList<3, 4, 5, 0x42>>; // 0x42 = temp
    using adc_i_t = adcController::index_type;

    using ppmDevPosition = void;
    
    //    using evrouter = Event::Router<Event::Channels<>, Event::Routes<>>;
    using evrouter = void;
    
    //    using eeprom = EEProm::Controller<Data>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl3Position, tcaPosition, servoPosition, sensorPosition, scanTermPosition>>;
    
    static inline void init() {
        portmux::init();
        if constexpr(!std::is_same_v<evrouter, void>) {
            //            evrouter::init();
        }
        ccp::unlock([]{
            clock::template init<Project::Config::fMcuMhz>();
        });
        systemTimer::init(); 
    }
    static inline void periodic() {
    }
};
