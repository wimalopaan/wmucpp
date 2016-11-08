/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#pragma once

#include "config.h"
#include "duration.h"

namespace Util {

#if !defined(__HAS_DELAY_CYCLES) || !defined(__OPTIMIZE__)
//# error "No builtin_avr_delay_cycles()"
#endif

__inline__ void delay(const std::microseconds&) __attribute__((__always_inline__));
__inline__ void delay(const std::milliseconds&) __attribute__((__always_inline__));

void delay(const std::microseconds& d) {
    __builtin_avr_delay_cycles(((F_CPU) / 1e6) * d.value);
}

void delay(const std::milliseconds& d) {
    __builtin_avr_delay_cycles(((F_CPU) / 1e3) * d.value);
}

}
