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

#include "util/concepts.h"

namespace Util {
template<std::Integral T, uint8_t Base = 10> constexpr uint8_t numberOfDigits();

namespace detail {
template<int Position, uint8_t Base = 10, std::Integral T = uint8_t>
uint8_t itoa_single(T& value, std::array<char, Util::numberOfDigits<T, Base>()>& data) {
    if constexpr(Position >= 0) {
        uint8_t fraction = value % Base;
        if (fraction < 10) {
            data[Position] = '0' + fraction;
        }
        else {
            data[Position] = 'a' - 10 + fraction;
        }
        value /= Base;
        return itoa_single<Position - 1, Base, T>(value, data);
    }
    return 0;
}

template<uint8_t Base = 10, std::Integral T = uint8_t, uint16_t L = 0, typename C = void>
auto itoa(const T& value, C& data) -> decltype(data)& {
    T v = value;
    if constexpr(std::is_signed<T>::value) {
        if (value < 0) {
            v = -value; 
        }
    }
    uint8_t position = std::numeric_limits<uint8_t>::max();
    do {
        uint8_t fraction = v % Base;
        if (fraction < 10) {
            data[++position] = '0' + fraction;
        }
        else {
            data[++position] = 'a' - 10 + fraction;
        }
        v /= Base;
    } while(v > 0);
    
    if constexpr(std::is_signed<T>::value) {
        if (value < 0) {
            data[++position] = '-';
        }
    }    
    data[position + 1] = '\0';    
    std::reverse(&data[0], &data[position]);
    return data;
}

template<uint8_t Position, typename T, uint16_t L>
auto ftoa(T& v, std::array<char, L>& data) -> decltype(data)& {
    typedef typename Util::fragmentType<T>::type FT;
    v *= 10;
    if (v != 0) {
        data[Position] = '0' + (v >> ((sizeof(FT)) * 8));
        v &= FT(-1);
        if constexpr(Position < data.size - 2) {
            return ftoa<Position + 1>(v, data);
        }
    }
    return data;    
}

} // detail
} // Util