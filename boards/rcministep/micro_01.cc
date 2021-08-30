#define NDEBUG

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

#include <external/hal/adccontroller.h>
#include <external/solutions/tick.h>
#include <external/solutions/series01/sppm_in.h>

#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

namespace  {
    using namespace External::Units::literals;
    constexpr auto fRtc = 8000_Hz; 
}

#include "stepper.h"
#include "gfsm.h"
#include "busses.h"
#include "devices.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using devices = Devices<Board412_01>;
using busdevs = BusDevs<External::Bus::NoBus<devices>>;

using gfsm = Bus::GFSM<busdevs>;

int main() {
    devices::portmux::init();
    devices::ccp::unlock([]{
#if (F_OSC == 20000000)
        devices::clock::prescale<1>();
#elif (F_OSC == 10000000)
        devices::clock::prescale<2>();
#else
#error "wrong F_OSC"
#endif
    });
    devices::systemTimer::init();
    
    gfsm::init();
    
    while(true) {
        gfsm::periodic();
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

