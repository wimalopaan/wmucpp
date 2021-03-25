//#define NDEBUG

#include <mcu/avr.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/sleep.h>

#include <external/solutions/cells.h>
#include <external/ibus/ibus.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/tick.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using activate = Pin<Port<A>, 1>; // and led
using conn12 = Pin<Port<A>, 7>;
using daisy = Pin<Port<A>, 2>;

//using txrx = Pin<Port<A>, 6>; // used for wake-up

// AIN3 = Voltage

namespace Parameter {
    constexpr auto fRtc = 1000_Hz;
    constexpr uint16_t R1vd = 100;
    constexpr uint16_t R2vd = 220;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Sensor
using portmux = Portmux::StaticMapper<Meta::List<usart1Position>>;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;

using telemetry = Hott::Experimental::Sensor<usart1Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, systemTimer>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::template init<Project::Config::fMcuMhz>();
    });
   
    systemTimer::init();
    
    telemetry::init();

    etl::copy(telemetry::text()[0], "WM 4S Sensor"_pgm);
    etl::copy(telemetry::text()[1], "Version 0.1"_pgm);

    
    while(true) {
        telemetry::periodic();
        systemTimer::periodic([&]{
            telemetry::ratePeriodic();
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
