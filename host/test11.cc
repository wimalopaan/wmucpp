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

#include <iostream>
#include <cassert>
#include <tuple>
//#include "util/meta.h"

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

namespace Meta {
    namespace detail {
        template<size_t N>
        struct visit {
            template<typename... T, typename F>
            static void at(const std::tuple<T...>& tuple, size_t index, F f) {
                if (index == (N - 1)) {
                    f(std::get<N - 1>(tuple));
                }
                else {
                    visit<N - 1>::at(tuple, index, f);
                }
            }
            
        };
        template<>
        struct visit<0> {
            template<typename... T, typename F>
            static void at(const std::tuple<T...>&, size_t, F) {
                assert(false);
            }
        };

    }
    template<typename... T, typename F>
    void visitAt(const std::tuple<T...>& tuple, size_t index, F f) {
        detail::visit<sizeof...(T)>::at(tuple, index, f);
    }
}

int main() {
    std::tuple<A, B, C> t;
    
    auto f = [](const auto& v) {
        v.f();
    };
    
    Meta::visitAt(t, 1, f);
    
}
