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

#include "util/dassert.h"
#include "util/util.h"

namespace std {

struct percent {
    constexpr explicit percent(uint8_t p) : mValue(p) {
        assert(mValue <= 100);
    }
    constexpr uint8_t value() const {
        return mValue;
    }
private:
    uint8_t mValue = 0;
};

constexpr bool operator==(std::percent lhs, std::percent rhs) {
    return lhs.value() == rhs.value();
}
constexpr bool operator!=(std::percent lhs, std::percent rhs) {
    return !(lhs == rhs);
}
constexpr bool operator>(std::percent lhs, std::percent rhs) {
    return lhs.value() > rhs.value();
}

namespace literals {
namespace quantity {

constexpr std::percent operator"" _ppc(unsigned long long v) {
    return std::percent{static_cast<uint8_t>(v)};
}

}
}

template<typename T>
constexpr percent scale(const T& value, const T& min, const T& max) {
    if (value < min) {
        return std::percent{0U};
    }
    else if (value > max) {
        return std::percent{100U};
    }
    else {
        return std::percent{(uint8_t)((static_cast<typename Util::enclosingType<T>::type>(value - min) * 100) / (max - min))};
    }
}

template<typename T>
constexpr T expand(percent p, const T& min, const T& max) {
    return min + ((max - min) * p.value()) / 100;
}


template<>
constexpr uint16_t expand<uint16_t>(percent p, const uint16_t& min, const uint16_t& max) {
    const uint16_t delta = max - min;
    const uint8_t deltaH = Util::upperHalf(delta);
    const uint8_t deltaL = Util::lowerHalf(delta);
    const uint16_t y = p.value() * deltaH;
    return min + (y << 1) + (y >> 1) + (y >> 4) - (y >> 9) + (p.value() * deltaL) / 100;
}


}
