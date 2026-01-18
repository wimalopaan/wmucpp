/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
// ATTENTION: in case of CRSF / SBUS / IBUS input is via PA1 (RX UART0) (PC1 (RX UART1) on hardware SSMSW01)
#define INPUT_CRSF // input via CRSF (ELRS only) (no rc-channel needed)
// #define INPUT_IBUS // input via IBUS (use subprotocol IBUS16 in 4in1-MPM) on channel 16
// #define INPUT_SBUS // input via SBUS (optional for ELRS, mandatory for other rc-link) on channel 16
// ATTENTION: in case of S.Port input is via PA0 (TX UART0 half-duplex)
// #define INPUT_SPORT // input via SPort (Phy-ID / App-ID see below)

#define DEFAULT_ADDRESS 0 // values: 0 ... 3 (must match value in widget)

#define USE_SSMSW01 // use the SuperSimpleMultiSwitch_01 hardware design (16 channel, telemetry, address pins)

// #define SBUS_INVERT // to receive SBus as "normal" uninverted serial data (invert SBUS means: invert an already inverted signal -> normal signal)

// use one(!) of the following options exclusively if input is via SBUS
#define USE_ELRS // SBus input only (see above)
// #define USE_AFHDS2A // SBus input only (see above) (using 4in1 MPM RF-module)
// #define USE_ACCST // SBus input only (see above) (using 4in1 MPM RF-module)

//#define DEBUG_OUTPUT // on attiny1614: CRSF/SBUS only: debug output via same uart as receiving data (same baudrate), S.Port is half-duplex -> no debug possible
// on avr128da28: uses USART1 (same as input on SSMSW01)

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

#define USE_ACTIVE_LOW_OUPUTS
#define USE_ADR_PINS
#ifndef DEBUG_OUTPUT
# define USE_TELEMETRY
#endif

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
#include <external/solutions/blinker.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hal/adccontroller.h>

#include <etl/output.h>
#include <etl/meta.h>
#include <etl/event.h>

#include "crsf.h"
#include "crsf_cb.h"
#include "sbus_cb.h"
#include "sport_cb.h"
#include "ibus_cb.h"
#include "leds.h"
#include "command.h"
#include "link.h"
#include "telemetry.h"
#include "stdcomp.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 2000_Hz; // timing 500µs resolution
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

# ifdef USE_SSMSW01

using led15 = Pin<Port<A>, 7>;
using led14 = Pin<Port<A>, 6>;
using led13 = Pin<Port<A>, 5>;
using led12 = Pin<Port<A>, 4>;
using led11 = Pin<Port<A>, 3>;
using led10 = Pin<Port<A>, 2>;
using led9  = Pin<Port<A>, 1>;
using led8  = Pin<Port<A>, 0>;

using tp = NoPin;

using ledPin  = Pin<Port<F>, 0>;
using led     = ActiveHigh<ledPin, Output>;
using in0Pin  = Pin<Port<F>, 6>;
using in0     = ActiveLow<in0Pin, Input>;
using aVBatt  = Pin<Port<F>, 1>;
using blinker = External::Blinker2<led, systemTimer, 100_ms, 2000_ms>;

using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<12>, AVR::Vref::V2_048>;
using adcController = External::Hal::AdcController<adc, Meta::NList<17, 0x42>>; // 0x42 = temp
using adc_i_t = adcController::index_type;

#ifdef USE_ADR_PINS
using adr0Pin = Pin<Port<C>, 2>;
using adr0    = ActiveLow<adr0Pin, Input>;
using adr1Pin = Pin<Port<C>, 3>;
using adr1    = ActiveLow<adr1Pin, Input>;
#endif
# else
// using tp = NoPin;
using tp   = Pin<Port<A>, 7>; // enables toggling this pin on every loop run
# endif

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

#ifdef USE_ACTIVE_LOW_OUPUTS
template<typename L>
using ledOutput = ActiveLow<L, Output>;
#else
template<typename L>
using ledOutput = ActiveHigh<L, Output>;
#endif

using ledList  = Meta::transform<ledOutput, Meta::List<led0, led1, led2, led3, led4, led5, led6, led7>>;

#ifdef USE_SSMSW01
using ledList2 = Meta::transform<ledOutput, Meta::List<led8, led9, led10, led11, led12, led13, led14, led15>>;
#endif

namespace Crsf {
    template<typename Config>
    struct Devices {
        using leds  = Leds<ledList>;
#ifdef USE_SSMSW01
        using leds2 = Leds<ledList2>;
#else
        using leds2 = void;
#endif
        struct CrsfAdapterConfig;
        using crsf_pa = Crsf::Adapter<CrsfAdapterConfig>;

#ifdef USE_SSMSW01
        using crsf = AVR::Usart<usart1Position, crsf_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
#else
        using crsf = AVR::Usart<usart0Position, crsf_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
#endif


#ifdef DEBUG_OUTPUT
#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
#  ifdef USE_SSMSW01
        using terminalDevice = crsf;
#  else
        using terminalDevice = AVR::Usart<usart1Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#  endif
#else
        using terminalDevice = crsf;
#endif
#else
        using terminalDevice = void;
#endif
        using terminal = etl::basic_ostream<terminalDevice>;

        using ledGroups = Meta::List<Devices::leds, Devices::leds2>;
        using commandCallback = CrsfCommandCallback<ledGroups>;

        struct TelemetryConfig;
        using telemetry = Telemetry<TelemetryConfig>;

        struct CommandDecoderConfig;
        using commandDecoder = Crsf::Command<CommandDecoderConfig>;

        struct LinkDecoderConfig;
        using linkDecoder = Crsf::Link<LinkDecoderConfig>;

        struct CrsfAdapterConfig {
            using debug = etl::basic_ostream<void>;
            using decoder = Meta::List<commandDecoder, linkDecoder>;
        };
        struct CommandDecoderConfig {
            using debug = etl::basic_ostream<void>;
            using callback = commandCallback;
        };
        struct LinkDecoderConfig {
            using debug = terminal;
            using callback = telemetry;
        };
        struct TelemetryConfig {
            using debug = terminal;
#ifdef DEBUG_OUTPUT
            using crsf = void;
#else
            using crsf = Devices::crsf;
#endif
        };

        enum class State : uint8_t {Undefined, Init, Connected, NotConnected};
        enum class Event : uint8_t {None, Connect, Unconnect};

        using components = StandardComponents<portmux, leds, leds2, in0, blinker,
        adcController, adr0, adr1, crsf, terminalDevice>;

        static inline void init() {
            components::init();
            tp::dir<Output>();
            crsf::template init<BaudRate<CRSF_BAUDRATE>>();

#if defined(__AVR_AVR128DA32__) or defined(__AVR_AVR128DA28__)
# ifdef DEBUG_OUTPUT
            if constexpr (!std::is_same_v<terminalDevice, crsf> && !std::is_same_v<terminalDevice, void>) {
                terminalDevice::template init<BaudRate<DEBUG_BAUDRATE>>();
            }
# endif
#endif
        }
        static inline void periodic() {
            tp::toggle();
            components::periodic();
        }
        static inline void ratePeriodic() {
            components::ratePeriodic();
            ++mStateTicks;
            (++mDebugTicks).on(debugTicks, [] static {
                       #ifdef USE_SSMSW01
                                   etl::outl<terminal>("adc0: "_pgm, adcController::value(adc_i_t{0}),
                                   "adc1: "_pgm, adcController::value(adc_i_t{1}));
                       #endif
                               });
            (++mCheckTicks).on(checkTicks, [] static {
                                   const uint8_t cp = crsf_pa::packages();
                                   if (cp > 0) {
                                       mEvent = Event::Connect;
                                   }
                                   else {
                                       mEvent = Event::Unconnect;
                                   }
                               });

            const auto oldState = mState;
            switch(mState) {
            case State::Undefined:
                mState = State::Init;
                break;
            case State::Init:
                mEvent.on(Event::Connect, []{
                    mState = State::Connected;
                }).thenOn(Event::Unconnect, []{
                    mState = State::NotConnected;
                });
                break;
            case State::Connected:
                mEvent.on(Event::Unconnect, []{
                    mState = State::NotConnected;
                });
                mStateTicks.on(telemetryTicks, []{
                    telemetry::voltage(adcController::value(adc_i_t{0}));
                    telemetry::temperature(adcController::value(adc_i_t{1}));
                    const uint8_t s = (in0::isActive() ? 0b0000'0001 : 0b0000'0000);
                    telemetry::status(s);
                });
                break;
            case State::NotConnected:
                mEvent.on(Event::Connect, []{
                    mState = State::Connected;
                });
                break;
            }
            if (oldState != mState) {
                mStateTicks.reset();
                switch(mState) {
                case State::Undefined:
                    break;
                case State::Init:
                    etl::outl<terminal>("Init"_pgm);
#ifdef USE_SSMSW01
                    blinker::steady();
#endif
                {
#ifdef USE_SSMSW01
# ifdef USE_ADR_PINS
                    const uint8_t a0 = adr0::isActive();
                    const uint8_t a1 = adr0::isActive();
                    const uint8_t address = DEFAULT_ADDRESS + (2 * a1 + a0) * 2;
                    commandDecoder::address(address);
# else
                    commandDecoder::address(DEFAULT_ADDRESS);
# endif
#else
                    commandDecoder::address(DEFAULT_ADDRESS);
#endif
                    etl::outl<terminal>("Adr: "_pgm, commandDecoder::address());
                }
                    break;
                case State::Connected:
                    etl::outl<terminal>("Connected"_pgm);
#ifdef USE_SSMSW01
                    blinker::blink(blinker::count_type{2});
#endif
                    break;
                case State::NotConnected:
                    etl::outl<terminal>("NotConnected"_pgm);
#ifdef USE_SSMSW01
                    blinker::blink(blinker::count_type{1});
#endif
                    break;
                }
            }
        }
        private:
        static constexpr External::Tick<systemTimer> debugTicks{500_ms};
        static constexpr External::Tick<systemTimer> telemetryTicks{100_ms};
        static constexpr External::Tick<systemTimer> checkTicks{3000_ms};
        inline static External::Tick<systemTimer> mStateTicks;
        inline static External::Tick<systemTimer> mCheckTicks;
        inline static External::Tick<systemTimer> mDebugTicks;
        inline static State mState = State::Undefined;
        inline static etl::Event<Event> mEvent{};
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
