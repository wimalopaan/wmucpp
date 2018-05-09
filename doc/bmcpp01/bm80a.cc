/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <stddef.h>

namespace {
     size_t counter{0};
}

class A {
public:
    A(uint8_t v) : mI{v} {}
    A(const A& rhs) : mI{rhs.mI} {}
    A& operator++() {
        ++mI;
        return *this;
    }
    A operator++(int) {
        A copy(*this);
        ++mI;
        return copy;
    }
    bool operator<(size_t rhs) const {
        return mI < rhs;
    }
private:
    uint8_t mI{0};
    size_t number{++counter};
};

volatile uint8_t x;

int main() {
    const size_t size = 10;
    for(A i = 0; i < size; ++i) { //<> weniger Aufwand
        (void)x;
    }
    for(A i = 0; i < size; i++) { //<> mehr Aufwand
        (void)x;
    }
    return counter;
}
