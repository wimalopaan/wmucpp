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

#pragma once

#include <ratio>

namespace Units {
    template<typename Representation, typename Divider = std::ratio<1,1>> struct frequency;
    
    using hertz = frequency<uint32_t, std::ratio<1, 1>>;
    using megahertz = frequency<uint16_t, std::ratio<1, 1'000'000>>;
     
    template<typename Representation, typename Divider>
    struct frequency {
        using divider_type = Divider;
        using value_type = Representation;
        const uint32_t value = 0;    
        constexpr operator hertz() const{
            return hertz(static_cast<uint32_t>(value) * Divider::den);
        }
    };
    
    template<typename Rep, typename Div>
    constexpr bool operator>=(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr) {
        return fl.value >= fr.value;
    }
    
    template<typename Rep, typename Div>
    constexpr uint32_t operator/(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr) {
        return fl.value / fr.value;
    }
    template<typename RepL, typename DivL, typename RepR, typename DivR>
    constexpr uint32_t operator/(const frequency<RepL, DivL>& fl, const frequency<RepR, DivR>& fr) {
        const hertz hl = fl;
        const hertz hr = fr;
        return hl / hr;
    }
    
    namespace literals {
        constexpr hertz operator ""_Hz(unsigned long long v) {
            return hertz{static_cast<uint32_t>(v)};
        }
        constexpr megahertz operator ""_MHz(unsigned long long v) {
            return megahertz{static_cast<uint8_t>(v)};
        }
    }
}
