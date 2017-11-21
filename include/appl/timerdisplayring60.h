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

#include <cstdint>
#include <chrono>

#include "timedisplays.h"

template<typename Leds, 
         const typename Leds::color_type& tickColor,
         const typename Leds::color_type& hourColor,
         const typename Leds::color_type& minuteColor,
         const typename Leds::color_type& secondColor
         >
class TimerDisplay60 final {
    TimerDisplay60() = delete;
public:
    typedef Leds leds;
    typedef typename Leds::color_type color_type;
    
    static constexpr uint8_t size = 60;
    static constexpr uint8_t hours = 12;
    static constexpr uint8_t minuteTick = 5;
    
    static_assert(Leds::size == size, "wrong number of leds");

    static void init() {
        Leds::init();
    }
    
    static void clear() {
        Leds::template set<false>(color_type{0});
    }

    template<typename Clock>
    static void set(const Clock& clock, TimeDisplay::Mode = TimeDisplay::Mode::Time) {
        DateTime::TimeTm t = clock.dateTime();
        Leds::template set<false>(color_type{0});
        for(uint8_t i = 0; i < size; ++i) {
            if ((i % minuteTick) == 0) {
                Leds::template set<false>(i, tickColor * mBrightness);
            }
        }
        
        Leds::template set<false>(((t.hours().value % hours * minuteTick) + (t.minutes().value * minuteTick) / size) % size, hourColor * mBrightness);
        Leds::template add<false>(t.minutes().value, minuteColor * mBrightness);
        Leds::template add<false>(t.seconds().value, secondColor * mBrightness);
        Leds::write();
    }
    static void brightness(const std::percent& b) {
        mBrightness = b;
    }

private:
    inline static std::percent mBrightness = 100_ppc;
};
