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

#include "container/pgmarray.h"

struct A {
    uint8_t m = 0;
    operator int() const {
        return m;
    }
    static A createFrom(std::array<std::byte, 1> b) {
        return A{uint8_t(b[0])};
    }
};

constexpr A a1{1};
constexpr A a2{2};
constexpr A a3{1};
constexpr A a4{2};
constexpr A a5{1};
constexpr A a6{2};
constexpr A a7{1};
constexpr A a8{2};
constexpr A a9{1};
constexpr A a10{2};

constexpr auto x1 = Util::PgmArray<const A&, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10>{};
constexpr auto x2 = Util::PgmArray<uint8_t, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10>{};

volatile uint8_t r = 0;

struct G {
    constexpr auto operator()() {
        std::array<uint8_t, 16> a;
        for(uint8_t i = 0; i < a.size; ++i) {
            a[i] = 2 * i + 1;
        }
        return a;
    }
};

using t = Util::Pgm::Converter<G>::pgm_type;
constexpr auto x3 = t{};

int main() {
//    for(const auto& v : x1) {
//        r += v;
//    }
//    for(const auto& v : x2) {
//        r += v;
//    }
    for(const auto& v : x3) {
        r += v;
    }
    return r;
}
