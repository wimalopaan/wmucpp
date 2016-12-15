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
//#include "std/initializer_list.h"

namespace std {

template<typename T, uint8_t Size>
struct array final
{
    typedef T type;
    typedef uint8_t size_type;
    
//    constexpr array() = default;
    
//    template<typename... TT>
//    constexpr array(TT&&... pp) : data{(T)pp...}{
//    }

    constexpr const T* begin() const {
        return &data[0];
    }
    constexpr const T* end() const {
        return &data[Size];
    }
    constexpr T* begin() {
        return &data[0];
    }
    constexpr T* end() {
        return &data[Size];
    }
    constexpr T& operator[](uint8_t index) {
        assert(index < Size);
        return data[index];
    }
    constexpr volatile T& operator[](uint8_t index) volatile {
        assert(index < Size);
        return data[index];
    }
    constexpr const T& operator[](uint8_t index) const {
        assert(index < Size);
        return data[index];
    }
    static constexpr uint8_t size = Size;
//private:
    T data[Size] = {};
};


}
