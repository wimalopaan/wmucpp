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
#include <type_traits>
#include <array>
#include <iostream>

namespace detail {
    template<class T>
    using maybe_cref = typename std::conditional<std::is_integral<T>::value, T, const T&>::type;
}

template<typename T, detail::maybe_cref<T>... Ts>
struct PgmArray final {
    inline static constexpr uint8_t size = sizeof... (Ts);
    inline static constexpr T data[] {Ts...};
};

struct A{
    uint8_t m = 0;
};
std::ostream& operator<<(std::ostream& o, const A& a) {
    return o << "A: " << (int)a.m;
}

constexpr A a1{1};
constexpr A a2{2};

constexpr auto x1 = PgmArray<A, a1, a2>{};
constexpr auto x2 = PgmArray<int, 1, 2>{};

int main() {
    for(auto& v : x1.data) {
        std::cout << v << '\n';        
    }
    for(auto& v : x2.data) {
        std::cout << v << '\n';        
    }
}
