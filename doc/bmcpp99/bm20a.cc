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

struct A {
    const uint8_t x = 0;
    const uint8_t y = 0;
};

static constexpr A a1{1, 2};
static constexpr A a2{2, 3};
static constexpr A a3{3, 4};

volatile uint8_t x;

struct B {
    static void foo(const A& a) {
        for(uint8_t i = 0; i < a.x; ++i) {
            x = a.y + i;
//            x = a.x + a.y + i;
        }
    }
};

int main() {
    B::foo(a1);
    B::foo(a2);
//    B::foo(a3);
    while(true);
}
