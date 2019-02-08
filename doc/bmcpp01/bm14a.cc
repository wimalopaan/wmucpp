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

#include <cstdint>
#include <cstddef> 
#include <cassert> 
#include <utility>
#include <avr/pgmspace.h>

using cppversion = std::integral_constant<long, __cplusplus>;
using gccversion = std::integral_constant<long, __GNUC__>;

//cppversion::_;
//gccversion::_;

struct Functor {
    constexpr char operator()(auto v) const {
        return 2 * v;
    }
};

auto l1 = [](auto v){return 2 * v;};

template<auto Size, typename F>
struct PgmArray {
    template<typename> struct Generator;
    using mapper = Generator<std::make_index_sequence<Size>>;
    inline static char value(size_t index) {
        assert(index < Size);
        return pgm_read_byte(&mapper::data[index]);
    }
    template<auto... Index>
    struct Generator<std::index_sequence<Index...>> {
        inline static constexpr char data[Size] PROGMEM = {
            F()(Index)... 
        }; 
    };
};

using a1 = PgmArray<10, Functor>;
using a2 = PgmArray<5, decltype(l1)>;
//using a2 = PgmArray<5, decltype([](auto v){return 2 * v;})>;

int main() {
    if constexpr((cppversion::value > 201703) && (gccversion::value > 8)) {
        return a1::value(2) + a2::value(1);
    }
    else {
        return a1::value(2);
    }
}
