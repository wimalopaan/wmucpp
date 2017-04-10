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

#pragma once

#include <stdint.h>

#include "std/time.h"
#include "units/percent.h"

#include "timedisplays.h"

// todo: Farben einbauen
template<typename Leds, const typename Leds::color_type& Color1>
class TimerDisplay4x4 final {
    TimerDisplay4x4() = delete;
public:
    typedef typename Leds::color_type Color;
    
    static constexpr uint8_t size = 16;
    static_assert(Leds::size == size, "wrong number of leds");

    static void init() {
        Leds::init();
    }

    template<typename Clock>
    static void set(const Clock& clock, TimeDisplay::Mode Mode = TimeDisplay::Mode::Time) {
        DateTime::TimeTm t = clock.dateTime();
        uint8_t min1   = t.minutes().value % 10;
        uint8_t min10  = t.minutes().value / 10;
        uint8_t hour1  = t.hours().value % 10;
        uint8_t hour10 = t.hours().value / 10;
        
        setNibble<0>(min1);
        setNibble<1>(min10);
        setNibble<2>(hour1);
        setNibble<3>(hour10);
        
        Leds::write();
    }
    
    template<uint8_t N>
    static void setNibble(uint8_t v) {
        if constexpr((N % 2) == 0) {
            for(uint8_t i = 0; i < 4; ++i) {
                if (v & (1 << i)) {
                    Leds::template set<false>(4 * N + i, Color1);
                }
                else {
                    Leds::template set<false>(4 * N + i, Color{0});
                }
            }
        }
        else {
            for(uint8_t i = 0; i < 4; ++i) {
                if (v & (1 << i)) {
                    Leds::template set<false>(4 * (N + 1) - 1 - i, Color1);
                }
                else {
                    Leds::template set<false>(4 * (N + 1) - 1 - i, Color{0});
                }
            }
        }
    }
    inline static std::percent brightness = 100_ppc;
};

