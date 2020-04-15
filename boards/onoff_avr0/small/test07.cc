#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/sleep.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/reset.h>
#include <mcu/internals/watchdog.h>
#include <mcu/common/uninitialized.h>
#include <mcu/pgm/pgmarray.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>
#include <external/ibus/ibus.h>
#include <external/units/music.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
    constexpr auto fRtc = 2000_Hz;
    
    constexpr uint16_t R1vd = 10'000;
    constexpr uint16_t R2vd = 1'000;

    constexpr uint16_t Rc = 1'000;
    constexpr uint16_t Kc = 16'450;
}

using reset = Reset<>;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using wdt   = WatchDog<systemTimer::intervall>;
using uninitialzed = Uninitialized<>;

// PA1: Voltage = AIN1
// PA2: Debug Pin
// PA3: Led
// PA4:
// PA5: current sense = AIN5
// PA6:
// PA7: Taster
// PB3:
// PB2: TXRX (Sensor)
// PB1: VN7003 = fet
// PB0: Buzzer = tca:wo0

using PortA = Port<A>;
using PortB = Port<B>;

using ledPin    = ActiveHigh<Pin<PortA, 3>, Output>;
using fet       = ActiveHigh<Pin<PortB, 1>, Output>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<1, 5, 0x1e>>; // 1e = temp

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

int main() {
//    wdt::init<ccp>();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    reset::noWatchDog([]{
        uninitialzed::value = 0x00_B;
    });
    
    fet::init();
    ledPin::init();

    portmux::init();
    systemTimer::init();
    adcController::init();
    
    terminalDevice::init<AVR::BaudRate<9600>>();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    uint8_t counter = 0;
    
    {
        if (std::any(uninitialzed::value)) {
            etl::outl<terminal>("**** WD Reset *****"_pgm);
        }
        else {
            etl::outl<terminal>("WD Test"_pgm);
        }
        while(true) {
            terminalDevice::periodic();
            adcController::periodic();
            
            systemTimer::periodic([&]{
                if (counter < 10) {
                    wdt::reset();
                }
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        ++counter;
                        etl::outl<terminal>("C: "_pgm, counter);
                    }
                });
            });
        }
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
