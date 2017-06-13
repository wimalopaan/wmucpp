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

template<uint8_t N, typename T>
struct A {
    static inline constexpr uint8_t number = N;
};

template<uint8_t N, typename T>
struct B {
    static inline constexpr uint8_t number = N;
};

template<typename I, typename... Pp>
struct T {
    
};


template<template <uint8_t> typename...  Rr>
struct ResCon {
    using X = std::make_index_sequence<sizeof...(Rr)>;
    
//    using X2 = T<X, Rr>;
}; 

struct P1{};
struct P2{};

template<uint8_t N>
using A1 = A<N, P1>;

template<uint8_t N>
using B1 = B<N, P2>;

using rc = ResCon<A1, B1>;

int main() {
    
}
