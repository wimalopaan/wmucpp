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

#include <stdint.h>

#include "std/optional.h"
#include "util/dassert.h"

namespace std {

template<typename T, uint8_t Capacity, typename CapType = uint8_t>
class FixedVector final {
public:
    // no iterator because of removal
    T& operator[](CapType index) {
        assert(index < Capacity);
        return data[index];
    }
    const T& operator[](CapType index) const {
        assert(index < Capacity);
        return data[index];
    }

    static constexpr const CapType capacity = Capacity;
    typedef CapType size_type;
    typedef T value_type;
    
    constexpr CapType size() const {
        return mSize;
    }

    // requires bool() Operator of elements
    std::optional<CapType> insert(const T& item) {
        for(CapType i = 0; i < Capacity; ++i) {
            if (!data[i]) {
                data[i] = item;
                ++mSize;
                return i;
            }
        }
        return {};
    }

    // preserve index
    void removeAt(CapType index) {
        assert(index < Capacity);
        data[index] = T();
        --mSize;
    }

private:
    T data[Capacity] = {};
    CapType mSize = 0;
};

}
