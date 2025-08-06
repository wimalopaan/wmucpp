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

#define NDEBUG // don't change (enables assertions)

// to modify the output-to-pin mapping, see below

// use one(!) of the following options exclusively
// ATTENTION: in case of CRSF / SBUS / IBUS input is via PA1 (RX UART0)
// #define INPUT_CRSF // input via CRSF (ELRS only) (no rc-channel needed)
#define INPUT_IBUS // input via IBUS (use subprotocol IBUS16 in 4in1-MPM) on channel 16
// #define INPUT_SBUS // input via SBUS (optional for ELRS, mandatory for other rc-link) on channel 16
// ATTENTION: in case of S.Port input is via PA0 (TX UART0 half-duplex)
// #define INPUT_SPORT // input via SPort (Phy-ID / App-ID see below)

#define DEFAULT_ADDRESS 0 // values: 0 ... 3 (must match value in widget)

// #define SBUS_INVERT // to receive SBus as "normal" uninverted serial data (invert SBUS means: invert an already inverted signal -> normal signal)

// use one(!) of the following options exclusively if input is via SBUS
#define USE_ELRS // SBus input only (see above)
// #define USE_AFHDS2A // SBus input only (see above) (using 4in1 MPM RF-module)
// #define USE_ACCST // SBus input only (see above) (using 4in1 MPM RF-module)

// #define DEBUG_OUTPUT // on attiny1614: CRSF/SBUS only: debug output via same uart as receiving data (same baudrate), S.Port is half-duplex -> no debug possible
                     // on avr128da28: uses USART1

#define CRSF_BAUDRATE 420'000
#define IBUS_BAUDRATE 115'200
#define SBUS_BAUDRATE 100'000
#define SPORT_BAUDRATE 57'600

#define DEBUG_BAUDRATE 115'200 // only availabe on MCU with more than one UART since S.Port is half-duplex

// Attention: activating S.Port response and if the sensor was discovered by EdgeTx, it
// is impossible to send data from EdgeTx to the sensor: LUA sportTelemetryPush() will fail in this case.
// So, be sure to delete the sensor prior to use the lvglMultiSwitchWidget!
// #define SPORT_NORESPONSE

#define SPORT_PHY External::SPort2::SensorId::ID1 // be aware, that ID1 equals 0 in the widget setup, ID2 equals 1, and so forth ...
#define SPORT_APP 0x51 // dec: 81

// ToDo:
// * byte-stuffing (S.Port) for incoming packets

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
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <etl/output.h>
#include <etl/meta.h>

#include "crsf.h"
#include "crsf_cb.h"
#include "sbus_cb.h"
#include "sport_cb.h"
#include "ibus_cb.h"
#include "leds.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 2000_Hz; // 500µs resolution
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;
using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position>>;
#else
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;
#endif

#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
// output-to-pin
using led7 = Pin<Port<D>, 7>;
using led6 = Pin<Port<D>, 6>;
using led5 = Pin<Port<D>, 5>;
using led4 = Pin<Port<D>, 4>;
using led3 = Pin<Port<D>, 3>;
using led2 = Pin<Port<D>, 2>;
using led1 = Pin<Port<D>, 1>;
using led0 = Pin<Port<D>, 0>;

// using tp = NoPin;
using tp   = Pin<Port<A>, 7>; // enables toggling this pin on every loop run

#elif defined(__AVR_ATtiny1614__)
// output-to-pin
using led7 = Pin<Port<A>, 4>;
using led6 = Pin<Port<A>, 5>;
using led5 = Pin<Port<A>, 6>;
using led4 = Pin<Port<A>, 7>;
using led3 = Pin<Port<A>, 1>;
using led2 = Pin<Port<A>, 2>;
using led1 = Pin<Port<B>, 1>;
using led0 = Pin<Port<B>, 0>;

using tp = NoPin;
// using tp   = Pin<Port<A>, 3>; // enables toggling this pin on every loop run
#endif

using ledList = Meta::List<led0, led1, led2, led3, led4, led5, led6, led7>;

namespace Crsf {
    template<typename Config>
    struct Devices {
        using leds = Leds<ledList>;

        struct CrsfAdapterConfig;
        using crsf_pa = Crsf::Adapter<CrsfAdapterConfig>;
        using crsf = AVR::Usart<usart0Position, crsf_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;

#ifdef DEBUG_OUTPUT
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
        using terminalDevice = AVR::Usart<usart1Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#else
        using terminalDevice = crsf;
#endif
#else
        using terminalDevice = void;
#endif
        using terminal = etl::basic_ostream<terminalDevice>;

        struct CrsfAdapterConfig {
            using debug = terminal;
            using cb = CrsfCommandCallback<leds>;
        };

        static inline void init() {
            portmux::init();
            leds::init();
            tp::dir<Output>();
            crsf_pa::address(std::byte{DEFAULT_ADDRESS});
            crsf::template init<BaudRate<CRSF_BAUDRATE>>();
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#ifdef DEBUG_OUTPUT
            terminalDevice::template init<BaudRate<DEBUG_BAUDRATE>>();
#endif
#endif
        }
        static inline void periodic() {
            tp::toggle();
            crsf::periodic();
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#ifdef DEBUG_OUTPUT
            terminalDevice::periodic();
#endif
#endif
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
namespace Ibus {
    template<typename Config>
    struct Devices {
        using leds = Leds<ledList>;

        struct CallbackConfig;
        using cb = IBusCommandCallback<CallbackConfig>;
        using ibus_pa = IBus2::Servo::ProtocollAdapter<0, void, cb>;
        using ibus = AVR::Usart<usart0Position, ibus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;

#ifdef DEBUG_OUTPUT
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
        using terminalDevice = AVR::Usart<usart1Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#else
        using terminalDevice = ibus;
#endif
#else
        using terminalDevice = void;
#endif
        using terminal = etl::basic_ostream<terminalDevice>;

        struct CallbackConfig {
            using debug = terminal;
            using leds = Devices::leds;
        };

        static inline void init() {
            portmux::init();
            leds::init();
            tp::dir<Output>();
            ibus::template init<BaudRate<IBUS_BAUDRATE>>();
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#ifdef DEBUG_OUTPUT
            terminalDevice::template init<BaudRate<DEBUG_BAUDRATE>>();
#endif
#endif
        }
        static inline void periodic() {
            // tp::toggle();
            ibus::periodic();
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#ifdef DEBUG_OUTPUT
            terminalDevice::periodic();
#endif
#endif
        }
        static inline void ratePeriodic() {
            ibus_pa::ratePeriodic();
            (++mStateTicks).on(debugTicks, [] static {
                etl::outl<terminal>("ibus p: "_pgm, ibus_pa::packages());
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

#ifdef DEBUG_OUTPUT
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
        using terminalDevice = AVR::Usart<usart1Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#else
        using terminalDevice = servo;
#endif
#else
        using terminalDevice = void;
#endif
        using terminal = etl::basic_ostream<terminalDevice>;

        struct CallbackConfig {
            using debug = terminal;
            using leds = Devices::leds;
        };
        static inline void init() {
            leds::init();
            tp::template dir<Output>();
            servo::template init<AVR::BaudRate<SBUS_BAUDRATE>, FullDuplex, true, 1>(); // 8E2
#ifndef SBUS_INVERT
            servo::rxInvert(true);
#endif
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#ifdef DEBUG_OUTPUT
            terminalDevice::template init<BaudRate<DEBUG_BAUDRATE>>();
#endif
#endif
        }
        static inline void periodic() {
            tp::toggle();
            servo::periodic();
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#ifdef DEBUG_OUTPUT
            terminalDevice::periodic();
#endif
#endif
        }
        static inline void ratePeriodic() {
            servo_pa::ratePeriodic();
            ++mStateTicks;
            mStateTicks.on(debugTicks, []{
#if defined(USE_ELRS)
            etl::outl<terminal>("elrs"_pgm);
#elif defined(USE_AFHDS2A)
            etl::outl<terminal>("afhds2a"_pgm);
#elif defined(USE_ACCST)
            etl::outl<terminal>("accst"_pgm);
#else
#error "wrong protocol"
#endif
            });
        }
        private:
        static constexpr External::Tick<systemTimer> debugTicks{500_ms};
        inline static External::Tick<systemTimer> mStateTicks;
    };
}
namespace SPort {
    template<typename Config>
    struct Devices {
        using leds = Leds<ledList>;

        struct CallbackConfig;
        using cb = SPortCommandCallback<CallbackConfig>;

        template<typename PA>
        using sensorUsart = AVR::Usart<usart0Position, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>>;
        struct Provider {
            inline static constexpr auto valueId = External::SPort::ValueId::DIY;
            inline static uint32_t value() {
                return 1234;
            }
        };
        using sensor = External::SPort2::Sensor<SPORT_PHY, sensorUsart, systemTimer, Meta::List<Provider>, cb>;

#ifdef DEBUG_OUTPUT
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
        using terminalDevice = AVR::Usart<usart1Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#else
        using terminalDevice = void;
#endif
#else
        using terminalDevice = void;
#endif
        using terminal = etl::basic_ostream<terminalDevice>;

        struct CallbackConfig {
            using debug = terminal;
            using leds = Devices::leds;
        };

        static inline void init() {
            leds::init();
            tp::template dir<Output>();
            sensor::init();
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#ifdef DEBUG_OUTPUT
            terminalDevice::template init<BaudRate<DEBUG_BAUDRATE>>();
#endif
#endif
        }
        static inline void periodic() {
            tp::toggle();
            sensor::periodic();
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#ifdef DEBUG_OUTPUT
            terminalDevice::periodic();
#endif
#endif
        }
        static inline void ratePeriodic() {
            sensor::ratePeriodic();
            ++mStateTicks;
            mStateTicks.on(debugTicks, []{
                etl::outl<terminal>("sport "_pgm, sensor::ProtocollAdapter::requests(), " "_pgm, sensor::ProtocollAdapter::commands(), " "_pgm, sensor::ProtocollAdapter::errors());
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
#ifdef INPUT_IBUS
using devices = Ibus::Devices<DevsConfig>;
#endif
#ifdef INPUT_SBUS
using devices = Sbus::Devices<DevsConfig>;
#endif
#ifdef INPUT_SPORT
using devices = SPort::Devices<DevsConfig>;
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
