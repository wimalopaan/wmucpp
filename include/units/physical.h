/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "std/ratio.h"
#include "units/duration.h"

namespace std {

template<typename Representation, typename Divider = ratio<1,1>>
struct frequency;

using hertz = frequency<uint32_t, ratio<1, 1>>;
using megahertz = frequency<uint8_t, ratio<1, 1000000>>;

template<>
struct frequency<uint32_t>
{
    typedef ratio<1, 1> divider_type;
    typedef uint32_t value_type;

    uint32_t value = 0;
};

template<>
struct frequency<uint8_t, ratio<1, 1000000>>
{
    typedef ratio<1, 1000000> divider_type;
    typedef uint8_t value_type;

    uint8_t value = 0;

    constexpr operator hertz() const{
        return {static_cast<uint32_t>(value * megahertz::divider_type::denom)};
    }
};

template<typename Rep, typename Div>
constexpr uint32_t operator/(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
{
    return fl.value / fr.value;
}

template<typename Rep, typename Div>
constexpr frequency<Rep, Div> operator/(const frequency<Rep, Div>& fl, Rep d) {
    return {(Rep)(fl.value / d)};
}

template<typename Rep, typename Div>
constexpr bool operator==(const frequency<Rep, Div>& lhs, const frequency<Rep, Div>& rhs) {
    return lhs.value == rhs.value;
}

template<typename Rep, typename Div>
constexpr bool operator!=(const frequency<Rep, Div>& lhs, const frequency<Rep, Div>& rhs) {
    return !(lhs == rhs);
}

template<typename Duration, typename Frequency>
constexpr uint32_t operator*(const Duration& dt, const Frequency& f) {
    return (dt.value * f.value * Frequency::divider_type::denom) / Duration::period_type::denom;
}

constexpr std::microseconds operator/(uint16_t v, const std::hertz& f) {
    return std::microseconds{(uint16_t)(((uint32_t)v * std::microseconds::period_type::denom) / f.value)};
}

template<typename T>
constexpr std::milliseconds duration_cast(const std::microseconds&);

template<>
constexpr std::milliseconds duration_cast<std::milliseconds>(const std::microseconds& us) {
    return std::milliseconds{(uint16_t)(us.value / 1000)};
}

}
