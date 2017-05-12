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
#include "std/array.h"

namespace Util {


namespace detail {
    template<uint8_t Digits = 2, uint8_t Base = 10> 
    struct LookupTable {
        typedef uint16_t dimension_type;
        constexpr inline static dimension_type dimension = Base * Base;
        
        constexpr static inline auto data = [](){
            std::array<std::array<char, Digits>, dimension> data;
            for(dimension_type i = 0; i < dimension; ++i) {
                auto value = i;
                for(int8_t d = Digits - 1; d >= 0; --d) {
                    auto r = value % Base;
                    if (r < 10) {
                        data[i][d] = '0' + r;
                    }
                    else {
                        data[i][d] = 'a' + r - 10;
                    }
                    value /= Base;
                }
            }
            return data;
        }();
    };






}
}
