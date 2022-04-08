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

#include <external/hott/generator.h>
#include <external/sbus/sbus.h>
#include <external/ibus/ibus2.h>

#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/rc/busscan.h>
#include <external/solutions/series01/sppm_out.h>
#include <external/solutions/spacemouse.h>

#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

#include "hc05.h"

namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units::literals;
    constexpr auto fRtc = 1000_Hz;
    
    
    struct Data final : public EEProm::DataBase<Data> {
        auto magic() const {
            return mMagic;
        }
        void clear() {
            mMagic = 42;   
        }
    private:
        uint8_t mMagic{};
    };
}

template<auto HWRev = 0, typename MCU = DefaultMcuType>
struct Devices;

template<typename MCU>
struct Devices<0, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, External::Bus::fRtc>;

    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; // sm1
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Default>; // serial2
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; // serial1
    using usart3Position = AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Default>; // sm2
    using usart4Position = AVR::Portmux::Position<AVR::Component::Usart<4>, AVR::Portmux::Default>; // BT
    
    using sm1_pa = External::SpaceMouse::ProtocollAdapter<0>;
    using sm1 = AVR::Usart<usart0Position, sm1_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;

    using sm2_pa = External::SpaceMouse::ProtocollAdapter<1>;
    using sm2 = AVR::Usart<usart3Position, sm2_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;

    using debug = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::C>, 2>, AVR::Output>; // serial2-ctr

    using sbus1 = External::SBus::Output::Generator<usart2Position, systemTimer, debug>;
    using sbus2 = External::SBus::Output::Generator<usart1Position, systemTimer>;

    using robo_pa = External::QtRobo::ProtocollAdapter<0>;
    using robo = AVR::Usart<usart4Position, robo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
    using terminal = etl::basic_ostream<robo>;
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 4>, AVR::Output>; 
    using btEnable = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::D>, 5>, AVR::Output>; 
    using btPower  = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::D>, 4>, AVR::Output>; 
  

#ifndef NDEBUG 
    using assertPin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 5>, AVR::Output>; // Pin1
#endif
    using test1Pin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 6>, AVR::Output>; // Pin2
    using test2Pin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 7>, AVR::Output>; // Pin3

    using ccl1Position = AVR::Portmux::Position<AVR::Component::Ccl<1>, AVR::Portmux::Default>; // PC3 = Serial2 TX
    using lut1 = AVR::Ccl::SimpleLut<1, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<1>>;

    using ccl2Position = AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Default>; // PD3 = Serial1 Tx
    using lut2 = AVR::Ccl::SimpleLut<2, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2>>;
    
//    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
          
    using eeprom = EEProm::Controller<Data>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl1Position, ccl2Position, usart0Position, usart1Position, usart2Position, usart3Position, usart4Position>>;
    
    static inline void init() {
        portmux::init();
//        if constexpr(!std::is_same_v<evrouter, void>) {
//            evrouter::init();
//        }
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDxSeries<MCU>) {
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

template<typename MCU>
struct Devices<1, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, External::Bus::fRtc>;

    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; // sm1
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Default>; // serial2
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; // serial1
    using usart3Position = AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Default>; // sm2
    using usart4Position = AVR::Portmux::Position<AVR::Component::Usart<4>, AVR::Portmux::Default>; // BT
    
    using sm1_pa = External::SpaceMouse::ProtocollAdapter<0>;
    using sm1 = AVR::Usart<usart0Position, sm1_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;

    using sm2_pa = External::SpaceMouse::ProtocollAdapter<1>;
    using sm2 = AVR::Usart<usart3Position, sm2_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;

//    using sbus1 = External::SBus::Output::Generator<usart2Position, systemTimer>;
    using sbus2 = External::SBus::Output::Generator<usart1Position, systemTimer>;
#if defined(USE_CTR_AS_DEBUG) 
    using debug = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::C>, 2>, AVR::Output>; // serial2-ctr
#endif
    
    using atbuffer = External::AT::Response<16>;
    using robo_pa = External::QtRobo::ProtocollAdapter<0, 16, atbuffer>;
//    using robo_pa = External::RoboRemo::ProtocollAdapter<0, 16, atbuffer>;
    using robo = AVR::Usart<usart4Position, robo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
    
    using dbg = AVR::Usart<usart2Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<8>, AVR::SendQueueLength<128>>;    
    using terminal = etl::basic_ostream<dbg>;
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 4>, AVR::Output>; 

    using btEnable = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::D>, 5>, AVR::Output>; 
    using btPower  = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::D>, 4>, AVR::Output>; 
    
    using hc05 = External::HC05<"abc"_pgm, robo, btEnable, btPower, systemTimer, terminal>;
    
#ifndef NDEBUG 
    using assertPin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 5>, AVR::Output>; // Pin1
#endif
    using test1Pin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 6>, AVR::Output>; // Pin2
    using test2Pin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 7>, AVR::Output>; // Pin3

    using ccl1Position = AVR::Portmux::Position<AVR::Component::Ccl<1>, AVR::Portmux::Default>; // PC3 = Serial2 TX
    using lut1 = AVR::Ccl::SimpleLut<1, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<1>>;

    using ccl2Position = AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Default>; // PD3 = Serial1 Tx
    using lut2 = AVR::Ccl::SimpleLut<2, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2>>;
    
//    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
          
    using eeprom = EEProm::Controller<Data>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl1Position, ccl2Position, usart0Position, usart1Position, usart2Position, usart3Position, usart4Position>>;
    
    static inline void init() {
#if defined(USE_CTR_AS_DEBUG)
        debug::init();
#endif
        portmux::init();
//        if constexpr(!std::is_same_v<evrouter, void>) {
//            evrouter::init();
//        }
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDxSeries<MCU>) {
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

template<typename MCU>
struct Devices<2, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, External::Bus::fRtc>;

    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; // sm1
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Default>; // serial2
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; // serial1
    using usart3Position = AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Default>; // sm2
    using usart4Position = AVR::Portmux::Position<AVR::Component::Usart<4>, AVR::Portmux::Default>; // BT
    
    using sm1_pa = External::SpaceMouse::ProtocollAdapter<0>;
    using sm1 = AVR::Usart<usart0Position, sm1_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;

    using sm2_pa = External::SpaceMouse::ProtocollAdapter<1>;
    using sm2 = AVR::Usart<usart3Position, sm2_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;

//    using sbus1 = External::SBus::Output::Generator<usart2Position, systemTimer>;
//    using sbus2 = External::SBus::Output::Generator<usart1Position, systemTimer>;
    using sumd = Hott::Output::GeneratorV3<usart1Position, systemTimer>;
#if defined(USE_CTR_AS_DEBUG) 
    using debug = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::C>, 2>, AVR::Output>; // serial2-ctr
#endif

    using atbuffer = External::AT::Response<16>;
    using robo_pa = External::QtRobo::ProtocollAdapter<0, 16, atbuffer>;
//    using robo_pa = External::RoboRemo::ProtocollAdapter<0, 16, atbuffer>;
    using robo = AVR::Usart<usart4Position, robo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
    using robo_out = etl::basic_ostream<robo>;
    
    using dbg = AVR::Usart<usart2Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<8>, AVR::SendQueueLength<128>>;    
    using terminal = etl::basic_ostream<dbg>;
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 4>, AVR::Output>; 

    using btEnable = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::D>, 5>, AVR::Output>; 
    using btPower  = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::D>, 4>, AVR::Output>; 
    
    using hc05 = External::HC05<"abc"_pgm, robo, btEnable, btPower, systemTimer, terminal>;
    
#ifndef NDEBUG 
    using assertPin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 5>, AVR::Output>; // Pin1
#endif
    using test1Pin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 6>, AVR::Output>; // Pin2
    using test2Pin = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 7>, AVR::Output>; // Pin3

    using ccl1Position = AVR::Portmux::Position<AVR::Component::Ccl<1>, AVR::Portmux::Default>; // PC3 = Serial2 TX
    using lut1 = AVR::Ccl::SimpleLut<1, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<1>>;

    using ccl2Position = AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Default>; // PD3 = Serial1 Tx
    using lut2 = AVR::Ccl::SimpleLut<2, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2>>;
    
//    using tcaPosition = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::Default>;
          
    using eeprom = EEProm::Controller<Data>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<ccl1Position, ccl2Position, usart0Position, usart1Position, usart2Position, usart3Position, usart4Position>>;
    
    static inline void init() {
#if defined(USE_CTR_AS_DEBUG)
        debug::init();
#endif
        portmux::init();
//        if constexpr(!std::is_same_v<evrouter, void>) {
//            evrouter::init();
//        }
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDxSeries<MCU>) {
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

