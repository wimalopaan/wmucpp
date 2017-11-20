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
#include "std/array"
#include "std/chrono"
#include "std/type_traits"
#include "std/concepts.h"

#include "timedisplays.h"

template<typename Spi>
class TimeBCDSerial {
public:
    static void init() {
        Spi::init();
        for(const auto& b: mData) {
            Spi::put(std::byte{b});
        }
    }

    template<std::Clock Clock>
    static void set(const Clock& clock, TimeDisplay::Mode Mode = TimeDisplay::Mode::Time) {
        DateTime::TimeTm t = clock.dateTime();
        if (Mode == TimeDisplay::Mode::Time) {
            auto hour   = t.hours().value;
            auto minute = t.minutes().value;
            auto seconds = t.seconds().value;
            
            mData[2] = ((hour % 10) << 4) + hour / 10;
            mData[1] = ((minute % 10) << 4) + minute / 10;
            mData[0] = ((seconds % 10) << 4) + seconds / 10;
        }
        else {
            auto year = (t.year().value - 2000) % 100;
            auto month = t.month().value;
            auto day = t.day().value;
            
            mData[2] = ((day % 10) << 4) + day/ 10;
            mData[1] = ((month % 10) << 4) + month / 10;
            mData[0] = ((year % 10) << 4) + year / 10;
        }
        
        for(const auto& b: mData) {
            Spi::put(std::byte{b});
        }
    }    
private:
    inline static std::array<uint8_t, 3> mData;
};
