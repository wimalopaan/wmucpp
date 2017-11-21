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

#include <stdint.h>
#include "util/algorithm.h"

volatile uint8_t n;

static uint8_t serial() {
    static uint8_t s = 0;
    return s++;
}

template<typename T = void>
struct A {
    A() : N(serial()) {
        n = N;
    }
    uint8_t N = 0;
};

template<typename T = void>
struct B {
    B() : N(serial()) {
        n = N;
    }
    uint8_t N = 0;
};

struct P1{};
struct P2{};

int main() {
    [[maybe_unused]] auto x1 = A<P1>();
    [[maybe_unused]] auto x2 = B<P2>();
    
    while(true) {}
}
