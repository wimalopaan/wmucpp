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

#define DEFAULT_ADDRESS 0 // must match value in widget
#define BAUDRATE 420'000

#include "board.h"
#include "crsf.h"
#include "crsf_cb.h"
#include "leds.h"

template<typename Config>
struct Devices {
    using leds = Leds<ledList>;

    struct CrsfAdapterConfig;
    using crsf_pa = Crsf::Adapter<CrsfAdapterConfig>;
    using crsf = AVR::Usart<usart0Position, crsf_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminalDevice = crsf;
    using terminal = etl::basic_ostream<terminalDevice>;

    struct CrsfAdapterConfig {
        using debug = terminal;
        using cb = CrsfCommandCallback<leds>;
    };

    static inline void init() {
        leds::init();
        crsf_pa::address(std::byte{DEFAULT_ADDRESS});
        crsf::template init<BaudRate<BAUDRATE>>();
    }
    static inline void periodic() {
        crsf::periodic();
    }
    static inline void ratePeriodic() {
        crsf_pa::ratePeriodic();
        ++mStateTicks;
        mStateTicks.on(debugTicks, []{
            etl::outl<terminal>("cp: "_pgm, crsf_pa::commandPackages());
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

