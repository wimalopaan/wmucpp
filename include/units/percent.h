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
#include "util/dassert.h"

namespace std {

struct percent {
    constexpr explicit percent(uint8_t p) : value(p) {
        assert(value <= 100);
    }
    uint8_t value = 0;
};

namespace literals {
namespace percent {


constexpr std::percent operator"" _ppc(unsigned long long v) {
    return std::percent{static_cast<uint8_t>(v)};
}

}
}

template<typename T>
constexpr percent scale(const T& value, const T& min, const T& max) {
    if (value < min) {
        return std::percent{0};
    }
    else if (value > max) {
        return std::percent{max};
    }
    else {
        return std::percent{(uint8_t)(((value - min) * 100) / (max - min))};
    }
}

// todo: 16bit: Aufteilen in MSB und LSB

template<typename T>
constexpr T expand(const percent& p, const T& min, const T& max) {
    return min + ((max - min) * p.value) / 100;
}


}
