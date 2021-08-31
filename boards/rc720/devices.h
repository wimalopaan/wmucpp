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
#include <external/solutions/series01/sppm_out.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

#include "esc.h"
#include "fbservo.h"
#include "internaltemp.h"
#include "adapter.h"

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
    
    template<typename Adc>
    struct Data final : public EEProm::DataBase<Data<Adc>> {
        auto magic() const {
            return mMagic;
        }
        auto channel() const {
            return mChannel;
        }
        void channel(const uint8_t c) {
            mChannel = c;
        }
        void clear() {
            mMagic = 42;   
            mChannel = 0;
        }
    private:
        uint8_t mMagic{};
        uint8_t mChannel{};
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

#ifdef NDEBUG
    using scanTermPosition = void;
    using scan_term_dev = void;
#else
    using scan_term_dev = External::TermSameAsScanDevice;
#endif
    
    using daisyChain= AVR::Pin<AVR::Port<AVR::D>, 4>; 
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::F>, 2>, AVR::Output>; // Fehler auf Platine
    using led2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::C>, 2>, AVR::Output>;
    
    using scanLedPin = led1;
#ifndef NDEBUG 
    using assertPin = led1;
#endif
    using configPin1 = AVR::Pin<AVR::Port<AVR::F>, 5>; // pwm4 = O1
    using configPin2 = AVR::Pin<AVR::Port<AVR::C>, 0>; // pwm5 = O2

    using config1 = AVR::ActiveLow<configPin1, AVR::Input>;
    using config2 = AVR::ActiveLow<configPin2, AVR::Input>;
    
    // lut2 out (tx2 inverted)
    using ccl2Position = AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Alt1>; // PD6
    using lut2 = AVR::Ccl::SimpleLut<2, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2>>;
    
    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
    
    using ibt = IBusThrough<daisyChain>;
    
    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<12>, AVR::Vref::V4_096>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<0, 19, 0x42>>; // 0x42 = temp
    using adc_i_t = adcController::index_type;
//    using tempiP = External::InternalTempProvider<adcController, 2, sigrow>;
    
    using pwm0_2 = External::Ppm::PpmOut<tcaPosition, Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>, 
                                         std::integral_constant<uint8_t, 16>>;
    
    using pwm3Position = AVR::Portmux::Position<AVR::Component::Tcb<0>, AVR::Portmux::Alt1>;
    using pwm4Position = AVR::Portmux::Position<AVR::Component::Tcb<1>, AVR::Portmux::Alt1>;
    using pwm5Position = AVR::Portmux::Position<AVR::Component::Tcb<2>, AVR::Portmux::Default>;

    using pwm3 = External::Ppm::PpmOut<pwm3Position, Meta::List<>, std::integral_constant<uint8_t, 16>>;
    using pwm4 = External::Ppm::PpmOut<pwm4Position, Meta::List<>, std::integral_constant<uint8_t, 16>>;
    using pwm5 = External::Ppm::PpmOut<pwm5Position, Meta::List<>, std::integral_constant<uint8_t, 16>>;
    
    using ppmDevPosition = void;

    using evch0 = AVR::Event::Channel<0, void>;
    using evch1 = AVR::Event::Channel<1, void>;
    using evch2 = AVR::Event::Channel<2, void>;
    using evuser0 = AVR::Event::Route<evch0, AVR::Event::Users::TcbCapt<0>>;
    using evuser1 = AVR::Event::Route<evch1, AVR::Event::Users::TcbCapt<1>>;
    using evuser2 = AVR::Event::Route<evch2, AVR::Event::Users::TcbCapt<2>>;
    using evrouter = AVR::Event::Router<AVR::Event::Channels<evch0, evch1, evch2>, AVR::Event::Routes<evuser0, evuser1, evuser2>>;
    
    using eeprom = EEProm::Controller<Data<adcController>>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl2Position, tcaPosition, servoPosition, sensorPosition, pwm3Position, pwm4Position, pwm5Position>>;
    
    static inline void init() {
        portmux::init();
        if constexpr(!std::is_same_v<evrouter, void>) {
            evrouter::init();
        }
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDa32<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
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

