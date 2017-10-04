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

#include <iostream>
#include <cassert>
#include "meta.h"

// 1) dynamische Polymorphie
// 2) switch 

// 3) List<> mit Visitor
// 4) List<> callAt / visitAt

// 5) Tuple<> mit Visitor
// 6) Tuple<> mit visitAt

struct A {
    static void f() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
};

struct B {
    static void f() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
};

struct C {
    static void f() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
};

using l1 = Meta::List<A, B, C>;

namespace Meta {
    namespace detail {
        template<typename L> struct visit_impl;
        
        template<template<typename ...> typename L, typename... T, typename First> 
        struct visit_impl<L<First, T...>> {
            template<typename F>
            static void at(size_t index, F f) {
                if (index == sizeof...(T)) {
                    f(First()); // Instanziierung
                }  
                else {
                    visit_impl<L<T...>>::at(index, f);
                }
            }
        };
        template<template<typename ...> typename L, typename... T> 
        struct visit_impl<L<T...>> {
            template<typename F>
            static void at(size_t, F) {
                assert(false);
            }
        };
    }
    template<typename L>
    struct visit {
        template<typename F>
        static void at(size_t index, F f) {
            detail::visit_impl<L>::at(size<L>::value - index - 1, f);    
        }
    };
}

int main() {
    constexpr auto f1 = [](auto x) {
        x.f();
    };
    Meta::visit<l1>::at(3, f1);
}
