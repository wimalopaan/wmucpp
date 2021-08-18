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

#include <external/solutions/rc/busscan.h>

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

using devices = Devices<Board1614_01>;
//using busdevs = BusDevs<External::Bus::IBusIBus<devices>>;
//using busdevs = BusDevs<External::Bus::SBusSPort<devices>>;
//using gfsm = Bus::GFSM<busdevs>;

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using busdevs = BusDevs<BusSystem>;
  
    inline static void timeout() {
        run(true); // defaults to SBUS
    }
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isIBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value) {
            using devs = busdevs::devs;
            using systemTimer = devs::systemTimer;
            using gfsm = Bus::GFSM<busdevs>;
            
            gfsm::init(inverted);
            while(true) {
                gfsm::periodic(); 
                systemTimer::periodic([&]{
                    gfsm::ratePeriodic();
                });
            }
        }
    }
};

using scanner = External::Scanner2<devices, Application, Meta::List<External::Bus::IBusIBus<devices>, External::Bus::SBusSPort<devices>>, AVR::FullDuplex, true>;

int main() {
    scanner::run();
}

