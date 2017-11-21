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

#define NDEBUG

#include "util/dassert.h"
#include "std/tuple"
#include "std/array"
#include "util/meta.h"

template<uint8_t N>
struct IndexNode {
    std::array<uint8_t, N> mChildren;
};

template<typename... T>
auto dSearch(const std::tuple<T...>& tuple) {
    
}

struct A {};
struct B {};
struct C {};
struct D {};
struct E {};

template<typename... T>
struct Node {
    constexpr Node(const T&... n) : mChildren{n...} {}
    std::tuple<T...> mChildren;
};




int main() {
    [[maybe_unused]] constexpr auto tree = []() {
        auto t = Node(A(), 
                       B(), 
                       Node(C(),
                            Node(D())), 
                       E());    
        
        // transform t into std::tuple<A, B, C, D, IndexNode<1>{3}, IndexNode<2>{2, 4}, E, IndexNode<4>{0, 1, 5, 6}>

        // return ...;        
        
        return t;
    }();
    
}
