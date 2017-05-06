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

#include "concepts.h"

namespace BCD {

template<typename T>
struct Digits;

template<>
struct Digits<uint8_t> {
    static constexpr uint8_t value = 3;
};
template<>
struct Digits<uint16_t> {
    static constexpr uint8_t value = 5;
};

template<typename To, Util::Array C>
To uconvert(const C& c) {
    To v = 0;
    for(uint8_t i = 0; (i < Digits<To>::value) && (i < c.size()); ++i) {
        uint8_t d = c[i] - '0';
        if (d < 10) {
            v = v * 10 + d;
        }
    }
    return v;
}


}