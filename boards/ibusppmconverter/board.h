#pragma once

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
#include <mcu/internals/event.h>
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
#include <external/solutions/series01/rpm.h>
#include <external/solutions/series01/sppm_out.h>

#include <std/chrono>
#include <etl/output.h>

//#define FS_I6S

// Config
// A        Sensor (I-Bus)
// ----
// A4t      4xTemperatur
// A2t2c    2xTemperatur, 2xStrom 
// A3t2r    3xTemperatur, 2xDrehzahl
// A2c2r    2xStrom, 2xDrehzahl
// A3c2r    3xStrom, 2xDrehzahl

// H        Sensor (Hott)

// S        Sensor (S.Port)

// B Pwm-Out
// ----
// B 10c5      5-Kanäle ab Kanal 10

// PA1: IBUS
// PA2: DaisyEnable
// PA3: SO4 (AIN3) (rpm0) (tcb1 wo)
// PA4:
// PA5: SO3 (AIN5) (tcb0 wo)
// PA6: 
// PA7: 
// PB3:
// PB2: Q0   (WO2) (rpm1)
// PB1: SO1  (WO1) (AIN10)
// PB0: SO2  (WO0) (AIN11)

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortB = Port<B>;

using so4Pin = Pin<PortA, 3>;
using so3Pin = Pin<PortA, 5>; // watch out for gps 
using q0Pin = Pin<PortB, 2>; 

using daisyChain= Pin<PortA, 2>; 

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

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;

template<typename Counter>
struct WdtProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::ARMED;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return Counter::counter;
    }
};

using wdtP = WdtProvider<uninitialzed>;

struct IBusThrough {
    inline static void init() {
        daisyChain::template dir<Output>();
    }
    inline static void on() {
        daisyChain::on();
    }
    inline static void off() {
        daisyChain::off();
    }
};

using ibt = IBusThrough;

