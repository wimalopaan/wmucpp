#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/twi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/dac.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>

#include <external/solutions/rotaryencoder.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/apa102.h>
#include <external/solutions/gps.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>

#include <external/sbus/sbus.h>
#include <external/ibus/ibus2.h>
#include <external/sbus/sport.h>

#include <external/hal/adccontroller.h>

#include <etl/meta.h>

#include "hc05.h"
#include "ds3231.h"
#include "ssd1306.h"

using namespace std::literals::chrono;
using namespace External::Units::literals;

template<auto HWRev = 0, typename MCU = DefaultMcuType>
struct Devices;

// HWRev 6
template<typename MCU>
struct Devices<5, MCU> {
    inline static constexpr megahertz fMcu{F_CPU / 1000000};
    inline static constexpr auto fRtc = 1000_Hz;
    
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;
    
    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; 
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Alt1>; 
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; 
    using usart3Position = AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Default>; 
    using usart4Position = AVR::Portmux::Position<AVR::Component::Usart<4>, AVR::Portmux::Default>; 
    
    using la0 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 4>, AVR::Output>; 
    using la1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 5>, AVR::Output>; 
    using la2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 6>, AVR::Output>; 
    using la3 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 7>, AVR::Output>; 
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 2>, AVR::Output>; 
    using led2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 3>, AVR::Output>; 
    using blinkLed1 = External::Blinker2<led1, systemTimer, 200_ms, 2000_ms>;
    using blinkLed2 = External::Blinker2<led2, systemTimer, 300_ms, 2000_ms>;

    using rmc = External::GPS::RMC<AVR::UseInterrupts<false>>;
    using vtg = External::GPS::VTG<AVR::UseInterrupts<false>>;
    using gsv = External::GPS::GSV<AVR::UseInterrupts<false>>;
    
    using gps_decoder_list = Meta::List<rmc, vtg, gsv>;
    using gps_pa  = External::GPS::GpsProtocollAdapter<0, gps_decoder_list>;
    using gps     = AVR::Usart<usart3Position, gps_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;

    using serial2 = AVR::Usart<usart4Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>>;
    using terminal2 = etl::basic_ostream<serial2>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, usart3Position, usart4Position>>;
    
    static inline void init() {
        portmux::init();
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDxSeriesAll<MCU>) {
                clock::template init<fMcu>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
    }
};

// HWRev 5
template<typename MCU>
struct Devices<4, MCU> {
    // ...
};

// HWRev 3
template<typename MCU>
struct Devices<2, MCU> {
    // ...
};

// HWRev 2
template<typename MCU>
struct Devices<1, MCU> {
    // ...
};

// HWRev 1
template<typename MCU>
struct Devices<0, MCU> {
    // ...
};

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    using terminal2 = devs::terminal2;
    using la1Scoped = AVR::ScopedPin<typename devs::la1>;
    
    enum State {Undefined, Init, On, On2, BT_SetName};
    
    static void init() {
        devs::init();
        // ...
        devs::gps::template init<AVR::BaudRate<9600>>();
        devs::serial2::template init<AVR::BaudRate<9600>>();
    } 
    static void periodic() {
        devs::la0::toggle();
        // ...
        devs::gps::periodic();
        devs::serial2::periodic();
        // fsm
    }
    static void ratePeriodic() {
        la1Scoped measure;
        
        devs::blinkLed1::ratePeriodic();
        devs::blinkLed2::ratePeriodic();

        // fsm        
        switch(mState) {
        case State::On:
            etl::outl<terminal2>("Bla"_pgm);                
        break;
            //...
        }
    }     
private:    
    static inline etl::StringBuffer<External::GPS::Sentence::TimeMaxWidth> timeBuffer;
    static inline State mState{State::Undefined};
    static inline External::Tick<typename devs::systemTimer> mStateTicks;
};

using devices = Devices<5>;
using gfsm = GlobalFsm<devices>;

int main() {
    gfsm::init();    
    while(true) {
        gfsm::periodic(); 
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
    
}
#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
