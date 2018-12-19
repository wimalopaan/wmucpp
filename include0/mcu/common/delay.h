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

#pragma once

//#include "units/duration.h"
//#include "std/literals.h"

#include <external/units/physical.h>

#if !defined(__BUILTIN_AVR_DELAY_CYCLES) || !defined(__OPTIMIZE__)
# ifndef __GLIBCXX__
#  error "No builtin_avr_delay_cycles()"
# endif
#endif

namespace AVR {
    namespace Util {
        using namespace External::Units::literals;
        using namespace std::chrono;
        
        __inline__ void delay(const centimicroseconds&) __attribute__((__always_inline__));
        __inline__ void delay(const microseconds&) __attribute__((__always_inline__));
        __inline__ void delay(const milliseconds&) __attribute__((__always_inline__));
        
        template<uint8_t Div>
        [[gnu::always_inline]] void delayNth(const microseconds& d) {
            __builtin_avr_delay_cycles((F_CPU / (1000000 * Div)) * d.value);
        }
        
        [[gnu::always_inline]] void delay(const centimicroseconds& d) {
            __builtin_avr_delay_cycles((F_CPU / (1000000 * 10)) * d.value);
        }
        
        [[gnu::always_inline]] void delay(const microseconds& d) {
            __builtin_avr_delay_cycles((F_CPU / 1000000) * d.value);
        }
        
        [[gnu::always_inline]] void delay(const milliseconds& d) {
            __builtin_avr_delay_cycles((F_CPU / 1000) * d.value);
        }
    }
}
