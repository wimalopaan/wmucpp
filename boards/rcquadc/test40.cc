//#define NDEBUG

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
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/apa102.h>
#include <external/solutions/rc/busscan.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

template<typename BusSystem, typename Timer, typename ServoUart>
struct GlobalFSM {
    inline static void init() {
        
    }
    inline static void periodic() {
        
    }
    inline static void ratePeriodic() {
        
    }
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;
    
    using servoPosition = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo / DBG
    using scanTermPosition = servoPosition;
    
    using sensorPosition = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Sensor
    using scanDevPosition = sensorPosition;

#ifdef NDEBUG
    using scan_term_dev = void;
#else
    using scan_term_dev = Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#endif

    using ppmDevPosition = void;  
//    using ppmDevPosition = AVR::Component::Tcb<0>;
//    using ppmIn =  AVR::Pin<AVR::Port<F>, 1>;
//    using ppm_channel = Event::Channel<4, Event::Generators::Pin<ppmIn>>; 
//    using ppm_user = Event::Route<ppm_channel, Event::Users::TcbCapt<0>>;
//    using evrouter = Event::Router<Event::Channels<ppm_channel>, Event::Routes<ppm_user>>;
    using evrouter = void;
    
    using scanLedPin = AVR::ActiveLow<NoPin, Output>;
    
    static inline void init() {
        using portmux = Portmux::StaticMapper<Meta::List<servoPosition, sensorPosition>>;
        portmux::init();
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
};
  
template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusSystem::devs;
    using periodic_dev = BusSystem::periodic_dev;
    using servo_pa = BusSystem::servo_pa;
    using terminal_device = BusSystem::terminal_device;
    using terminal = etl::basic_ostream<terminal_device>;
    using systemTimer = devs::systemTimer;
    
    using gfsm = GlobalFSM<BusSystem, systemTimer, periodic_dev>;
    
    [[noreturn]] inline static void run(const bool inverted = false) {
        gfsm::init();
        etl::outl<terminal>("test40: "_pgm, GITTAG_PGM);
        while(true) {
            gfsm::periodic(); 
            systemTimer::periodic([&]{
                gfsm::ratePeriodic();
            });
        }
    }
};

using devices = Devices<>;
using scanner = External::Scanner<devices, Application, AVR::HalfDuplex>;

int main() {
    scanner::run();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
//    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
//        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
