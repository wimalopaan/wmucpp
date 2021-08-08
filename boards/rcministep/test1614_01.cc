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

#include "stepper.h"
#include "gfsm.h"
#include "busses.h"

namespace  {
    using namespace External::Units::literals;
    constexpr auto fRtc = 8000_Hz; 
}

#include "devices.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using devices = Devices<NoBoard1614>;
using busdevs = BusDevs<External::Bus::IBusIBus<devices>>;

using gfsm = Bus::GFSM<busdevs>;

int main() {
    devices::portmux::init();
    devices::ccp::unlock([]{
        devices::clock::prescale<1>();
    });
    devices::systemTimer::init();
    devices::evrouter::init();
    
    gfsm::init();
    
    while(true) {
        gfsm::periodic();
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

