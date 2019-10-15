/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <limits>
#include <array>
#include "std/concepts"
#include "util/bits.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "hal/constantrate.h"
#include "util/types.h"
#include "util/disable.h"
#include "util/bits.h"
#include "util/rational.h"
#include "container/stringbuffer.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

volatile uint16_t y = 42;
//constexpr uint16_t y = 42;

//using D = Util::RationalDivider<uint16_t, 2500, 6400>;
using D = Util::RationalDivider<uint16_t, 8, 10>;


int main() {
    std::outl<terminal>(D::N);
    std::outl<terminal>(D::D);
    std::outl<terminal>(D::GCD);
    std::outl<terminal>(D::data.shifts);
    std::outl<terminal>(D::data.multiplierFull);
    std::outl<terminal>(D::data.multiplierTruncated);
    std::outl<terminal>(D::scale(y));
}

