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

// This file is for the old multiswitch PCBs
// The pinout of the outputs is defined in "board.h"

#define NDEBUG

#define DEFAULT_ADDRESS 0 // (values: 0 ... 3) must match address value in edgetx widget config

// use one(!) of the following options exclusively
#define USE_ELRS // sbus input over ELRS rc-link
// #define USE_AFHDS2A // sbus input over AFHDS2A rc-link (using 4in1 MPM RF-module)
// #define USE_ACCST // sbus input over ACCST rc-link (using 4in1 MPM RF-module)

// #define DEBUG_OUTPUT

#define SBUS_IBUS_NO_WARN
#include "board.h"
#include "leds.h"
#include "sbus_cb.h"

template<typename Config>
struct Devices {
    using leds = Leds<ledList>;

    struct CallbackConfig;
    using cb = SBusCommandCallback<CallbackConfig>;
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer, void, cb>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifdef DEBUG_OUTPUT
    using terminalDevice = servo;
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
        servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
        servo::rxInvert(true);
    }
    static inline void periodic() {
        servo::periodic();
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

struct DevsConfig {};
using devices = Devices<DevsConfig>;

int main() {
    portmux1::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();

    devices::init();
    while(true) {
        devices::periodic();
        systemTimer::periodic([&]{
            devices::ratePeriodic();
        });
    }
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<devices::terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
   while(true) {
//        dbg1::toggle();
   }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif

