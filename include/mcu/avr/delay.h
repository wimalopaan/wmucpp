/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "units/duration.h"

namespace Util {

#if !defined(__BUILTIN_AVR_DELAY_CYCLES) || !defined(__OPTIMIZE__)
# ifndef __GLIBCXX__
#  error "No builtin_avr_delay_cycles()"
# endif
#endif

__inline__ void delay(const std::centimicroseconds&) __attribute__((__always_inline__));
__inline__ void delay(const std::microseconds&) __attribute__((__always_inline__));
__inline__ void delay(const std::milliseconds&) __attribute__((__always_inline__));

#ifdef __AVR__

template<uint8_t Div>
void delayNth(const std::microseconds& d) {
    __builtin_avr_delay_cycles((F_CPU / (1000000 * Div)) * d.value);
}

void delay(const std::centimicroseconds& d) {
    __builtin_avr_delay_cycles((F_CPU / (1000000 * 10)) * d.value);
}

void delay(const std::microseconds& d) {
    __builtin_avr_delay_cycles((F_CPU / 1000000) * d.value);
}

void delay(const std::milliseconds& d) {
    __builtin_avr_delay_cycles((F_CPU / 1000) * d.value);
}
#endif

}
