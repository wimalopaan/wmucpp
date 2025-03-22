/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define NDEBUG

#define INPUT_CRSF
// #define INPUT_SBUS

#define DEFAULT_ADDRESS 0

#define CRSF_BAUDRATE 420'000
#define SBUS_BAUDRATE 100'000

// #define SBUS_INVERT

#define USE_ELRS
//#define USE_AFHDS2A
//#define USE_FRSKY

#define DEBUG_OUTPUT

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>

#include <external/solutions/tick.h>
#include <external/sbus/sbus.h>

#include <etl/output.h>
#include <etl/meta.h>

#include "crsf.h"
#include "crsf_cb.h"
#include "sbus_cb.h"
#include "leds.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;
using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

#if defined(__AVR_AVR128DA32__)
using led7 = Pin<Port<D>, 7>;
using led6 = Pin<Port<D>, 6>;
using led5 = Pin<Port<D>, 5>;
using led4 = Pin<Port<D>, 4>;
using led3 = Pin<Port<D>, 3>;
using led2 = Pin<Port<D>, 2>;
using led1 = Pin<Port<D>, 1>;
using led0 = Pin<Port<D>, 0>;

using tp   = Pin<Port<A>, 7>;

#elif defined(__AVR_ATtiny1614__)
using led7 = Pin<Port<A>, 4>;
using led6 = Pin<Port<A>, 5>;
using led5 = Pin<Port<A>, 6>;
using led4 = Pin<Port<A>, 7>;
using led3 = Pin<Port<A>, 1>;
using led2 = Pin<Port<A>, 2>;
using led1 = Pin<Port<B>, 1>;
using led0 = Pin<Port<B>, 0>;

using tp   = Pin<Port<A>, 3>;
#endif


using ledList = Meta::List<led0, led1, led2, led3, led4, led5, led6, led7>;

namespace Crsf {
    template<typename Config>
    struct Devices {
        using leds = Leds<ledList>;

        struct CrsfAdapterConfig;
        using crsf_pa = Crsf::Adapter<CrsfAdapterConfig>;
        using crsf = AVR::Usart<usart0Position, crsf_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    #ifdef DEBUG_OUTPUT
        using terminalDevice = crsf;
    #else
        using terminalDevice = void;
    #endif
        using terminal = etl::basic_ostream<terminalDevice>;

        struct CrsfAdapterConfig {
            using debug = terminal;
            using cb = CrsfCommandCallback<leds>;
        };

        static inline void init() {
            leds::init();
            tp::dir<Output>();
            crsf_pa::address(std::byte{DEFAULT_ADDRESS});
            crsf::template init<BaudRate<CRSF_BAUDRATE>>();
        }
        static inline void periodic() {
            tp::toggle();
            crsf::periodic();
        }
        static inline void ratePeriodic() {
            crsf_pa::ratePeriodic();
            (++mStateTicks).on(debugTicks, [] static {
                etl::outl<terminal>("cp: "_pgm, crsf_pa::commandPackages());
            });
        }
        private:
        static constexpr External::Tick<systemTimer> debugTicks{500_ms};
        inline static External::Tick<systemTimer> mStateTicks;
    };
}

namespace Sbus {
    template<typename Config>
    struct Devices {
        using leds = Leds<ledList>;

        struct CallbackConfig;
        using cb = SBusCommandCallback<CallbackConfig>;
        using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer, void, cb>;
        using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

        using terminalDevice = servo;
        using terminal = etl::basic_ostream<terminalDevice>;

        struct CallbackConfig {
            using debug = terminal;
            using leds = Devices::leds;
        };
        static inline void init() {
            leds::init();
            tp::dir<Output>();
            servo::template init<AVR::BaudRate<SBUS_BAUDRATE>, FullDuplex, true, 1>(); // 8E2
#ifndef SBUS_INVERT
            servo::rxInvert(true);
#endif
        }
        static inline void periodic() {
            tp::toggle();
            servo::periodic();
        }
        static inline void ratePeriodic() {
            servo_pa::ratePeriodic();
            ++mStateTicks;
            mStateTicks.on(debugTicks, []{
                etl::outl<terminal>("tick"_pgm);
            });
        }
        private:
        static constexpr External::Tick<systemTimer> debugTicks{500_ms};
        inline static External::Tick<systemTimer> mStateTicks;
    };

}

struct DevsConfig {};
#ifdef INPUT_CRSF
using devices = Crsf::Devices<DevsConfig>;
#endif
#ifdef INPUT_SBUS
using devices = Sbus::Devices<DevsConfig>;
#endif

int main() {
    portmux::init();
    ccp::unlock([]{
#if defined(__AVR_AVR128DA32__)
        clock::template init<Project::Config::fMcuMhz>();
#elif defined(__AVR_ATtiny1614__)
        clock::prescale<2>();
#else
#error "wrong MCU"
#endif
    });
    systemTimer::init();

    devices::init();
    while(true) {
        devices::periodic();
        systemTimer::periodic([] static {
            devices::ratePeriodic();
        });
    }
}
