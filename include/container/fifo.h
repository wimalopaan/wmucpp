/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "std/optional.h"
#include "std/traits.h"

namespace std {

template<typename T, uint16_t Size = 32>
class FiFo final {
public:
    typedef typename std::conditional< Size <= 255, uint8_t, uint16_t>::type size_type;
    
    bool push_back(const T& item) {
        size_type next = (in + 1) % Size;
        if (out == next) {
            return false;
        }
        data[in] = item;
        in = next;
        return true;
    }
    bool pop_front(T& item) {
        if (in == out) {
            return false;
        }
        item = data[out];
        out = (out + 1) % Size;
        return true;
    }
    std::optional<T>  pop_front() {
        if (in == out) {
            return {};
        }
        T item = data[out];
        out = (out + 1) % Size;
        return item;
    }
    void clear() {
        in = out = 0;
    }
    bool empty() const {
        return in == out;
    }
    static constexpr const size_type size = Size;
private:
    T data[Size] = {};
    size_type in = 0;
    size_type out = 0;
};

}
