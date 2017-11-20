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
#include <type_traits>
#include <ratio>
#include <limits>

#include "std/concepts.h"
#include "units/duration.h"

namespace std {

template<typename Representation, typename Divider = ratio<1,1>>
struct frequency;

using hertz = frequency<uint32_t, ratio<1, 1>>;
using megahertz = frequency<uint8_t, ratio<1, 1000000>>;

template<>
struct frequency<uint32_t> {
    typedef ratio<1, 1> divider_type;
    typedef uint32_t value_type;

    const uint32_t value = 0;
};

template<>
struct frequency<uint8_t, ratio<1, 1000000>> {
    typedef ratio<1, 1000000> divider_type;
    typedef uint8_t value_type;

    const uint8_t value = 0;

    constexpr operator hertz() const{
        return {static_cast<uint32_t>(value * megahertz::divider_type::denom)};
    }
};

template<typename Rep, typename Div, Integral I>
constexpr frequency<Rep, Div> operator*(I i, const frequency<Rep, Div>& f) {
    return {f.value * i};
}
template<typename Rep, typename Div, Integral I>
constexpr frequency<Rep, Div> operator*(const frequency<Rep, Div>& f, I i) {
    return {f.value * i};
}

template<typename Rep, typename Div>
constexpr uint32_t operator/(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
{
    return fl.value / fr.value;
}

template<typename Rep, typename Div>
constexpr bool operator<(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
{
    return fl.value < fr.value;
}

template<typename Rep, typename Div>
constexpr bool operator>=(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
{
    return !(fl.value < fr.value);
}

template<typename Rep, typename Div>
constexpr bool operator<=(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
{
    return fl.value <= fr.value;
}

template<typename Rep, typename Div, Integral I>
constexpr frequency<Rep, Div> operator/(const frequency<Rep, Div>& fl, I d) {
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

constexpr std::hertz operator/(uint16_t v, const std::microseconds& f) {
    uint32_t x = v;
    uint32_t fl = f.value;
    return {(uint32_t)((x * std::microseconds::period_type::denom) / fl)};
}

template<typename T>
constexpr std::milliseconds duration_cast(const std::microseconds&);

template<>
constexpr std::milliseconds duration_cast<std::milliseconds>(const std::microseconds& us) {
    return std::milliseconds{(uint16_t)(us.value / 1000)};
}

template<typename T>
constexpr std::microseconds duration_cast(const std::milliseconds&);
template<>
constexpr std::microseconds duration_cast<std::microseconds>(const std::milliseconds& ms) {
    return std::microseconds{(uint16_t)(ms.value * 1000)};
}

class RPM {
public:
    explicit constexpr RPM() : mValue(std::numeric_limits<uint16_t>::max()) {}
    explicit constexpr RPM(uint16_t v) : mValue{v} {}
    explicit constexpr RPM(const std::hertz& f) : mValue(f.value * 60) {}
    
    constexpr explicit operator bool() const {
        return mValue != std::numeric_limits<uint16_t>::max();        
    }
    constexpr uint16_t value() const {
        if (mValue < std::numeric_limits<uint16_t>::max()) {
            return mValue;
        }
        else {
            return 0;
        }
    }
private:
    uint16_t mValue = 0;
};


}
