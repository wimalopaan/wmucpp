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
#include "util/algorithm.h"

volatile uint8_t n;

template<uint8_t N, typename T = void>
struct A {
    typedef T  type;
    static inline constexpr uint8_t number = N;
    static void init() {n = N;}
};

template<uint8_t N, typename T = void>
struct B {
    typedef T  type;
    static inline constexpr uint8_t number = N;
    static void init() {n = N;}
};

struct P1{};
struct P2{};

template<uint8_t N>
using A1 = A<N, P1>;

template<uint8_t N>
using B1 = B<N, P2>;

template <template <int> class... Z>
struct SeqInst {
    inline static constexpr uint8_t Size = sizeof...(Z);
    using Indexes = std::make_index_sequence<sizeof...(Z)>;
    static void init() {
        init(Indexes{});
    }
private:
    template<typename... T>
    struct TypeList {
        static void init() {
            (T::init(), ...);
        }
    };

    template <size_t... Is>
    static void init(std::index_sequence<Is...> ) {
        using tl = TypeList<Z<Is>...>;        
        tl::init();
    }
};

using test = SeqInst<A1, B1>;

int main() {
    test::init();
    
    while(true) {}
}
