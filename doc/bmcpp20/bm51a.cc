/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>
#include "util/bits.h"
#include "std/limits"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "hal/constantrate.h"
#include "std/array"
#include "std/concepts.h"
#include "util/disable.h"
#include "util/bits.h"
#include "util/rational.h"
#include "container/stringbuffer.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

volatile uint8_t y = 42;
volatile uint8_t z = 0;
volatile uint16_t z2 = 42;
volatile uint32_t z3 = 0;

// multiply by 100/255
constexpr uint8_t scale1(uint8_t value) {
    const uint8_t s = 201;
    return (value * s) / 256 / 2;
}

constexpr uint8_t scale2(uint8_t value) {
    return (value * 100u) / 255u;
}


int main() {
    y = 42;
    {
        z = scale1(y);
    }
//    y = 43;
//    {
//        z = scale2(y);
//    }
//    y = 44;
    {
        auto x = Util::uint_scaled<uint16_t, 100, 255>(z2);
        z2 = x.toInt();
        std::outl<terminal>(z2);
    }
//    {
//        z2 = scale3(z2).toInt();
//        std::outl<terminal>(z2);
//    }
//    {
//        auto a = uint_scaled<uint8_t, 100, 255>{y};
//        z = a.toInt();
//    }
//    {
//        z = scale4<100, 255>(y);
//    }
//    {
        auto a = Util::uint_scaled<uint8_t, 100, 255>{255};
        auto b = Util::uint_scaled<uint8_t, 100, 255>{255};
        auto c = a * b;
        z2 = c.toInt();
        std::outl<terminal>(z2);
//    }
    while(true) {}
}

