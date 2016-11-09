/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

template<typename T, uint8_t Capacity>
class FixedVector final {
public:
    // no iterator because of removal
    T& operator[](uint8_t index) {
        assert(index < Capacity);
        return data[index];
    }
    const T& operator[](uint8_t index) const {
        assert(index < Capacity);
        return data[index];
    }

    static constexpr const uint8_t capacity = Capacity;

    constexpr uint8_t size() const {
        return mSize;
    }

    // requires bool() Operator of elements
    std::optional<uint8_t> insert(const T& item) {
        for(uint8_t i = 0; i < Capacity; ++i) {
            if (!data[i]) {
                data[i] = item;
                ++mSize;
                return i;
            }
        }
        return {};
    }

    // preserve index
    void removeAt(uint8_t index) {
        assert(index < Capacity);
        data[index] = T();
        --mSize;
    }

private:
    T data[Capacity] = {};
    uint8_t mSize = 0;
};

}
