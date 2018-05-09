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

#define NDEBUG

#include <tuple>

template<auto... II> struct IndexNode {};

struct A {uint8_t v{};};
struct B {uint8_t v{};};
struct C {uint8_t v{};};
struct D {uint8_t v{};};
struct E {uint8_t v{};};

template<typename... CC>
struct Node {
    constexpr Node(const CC&... n) : mChildren{n...} {}
    std::tuple<CC...> mChildren;
};

constexpr auto transform1(const auto& tree) {
    constexpr auto first = std::get<0>(tree.mChildren); // nicht möglich
    return IndexNode<first.v>{};    
}
constexpr auto transform2(const auto& callable) {
    constexpr auto tree = callable();
    constexpr auto first = std::get<0>(tree.mChildren);
    return IndexNode<first.v>{};
}

constexpr auto foo(const auto& v) {return v;} 

int main() {
    constexpr auto tree = []{
        auto t1 = Node(A{1}, 
                       B{2}, 
                       Node(C{3},
                            Node(D{4})), 
                       E{5});    
        auto t2 = foo(t1); // hier ohne Belang, nur zur Demo der IIFE
        return t2;
    }();

//    constexpr auto flat1 = transform1(tree); // nicht möglich

    [[maybe_unused]] constexpr auto flat2 = transform2([&]{return tree;}); // constexpr lambda argument wrapper
}
