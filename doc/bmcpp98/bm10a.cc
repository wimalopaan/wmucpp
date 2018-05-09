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

#include "std/type_traits"

constexpr bool useA1 = true;
constexpr bool useB = true;

struct A1 {
    static void f() {
        x = 1;
    }
    inline static volatile uint8_t x = 0;
};
struct A2 {
    static void f() {
        x = 2;
    }
    inline static volatile uint8_t x = 0;
};
struct B {
    static void g() {
        x = 3;
    }
    inline static volatile uint8_t x = 0;
};

using a = std::conditional<useA1, A1, A2>::type;
using b = std::conditional<useB, B, void>::type;

namespace detail {
template<typename AT, typename BT>
void main() {
    AT::f();
    
    if constexpr(std::is_same<BT, B>::value) {
        BT::g();
    }
    
    while(true) {}
}

} //!detail

int main() {
    detail::main<a, b>();
}
