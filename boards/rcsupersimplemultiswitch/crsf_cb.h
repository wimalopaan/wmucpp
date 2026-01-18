/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "etl/meta.h"

#include "crsf.h"

template<typename LedGroups>
struct CrsfCommandCallback {
    using ledGroups = LedGroups;
    using ledsAdr0 = Meta::nth_element<0, ledGroups>;

    static inline void set(const std::byte data) {
        std::byte mask = 0b1_B;
        for(uint8_t i = 0; i < 8; ++i) {
            if (std::any(data & mask)) {
                ledsAdr0::set(i, true);
            }
            else {
                ledsAdr0::set(i, false);
            }
            mask <<= 1;
        }
    }
    static inline void setIndex(const int8_t adrIndex, const uint8_t i, const bool on) {
        if (adrIndex == 0) {
            ledsAdr0::set(i, on);
        }
        else if (adrIndex == 1) {
            if constexpr(Meta::size_v<ledGroups> > 0) {
                using ledsAdr1 = Meta::nth_element<1, ledGroups>;
                ledsAdr1::set(i, on);
            }

        }
    }
};
