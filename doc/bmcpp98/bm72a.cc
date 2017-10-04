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
#include "std/tuple.h"
#include "std/array.h"
#include "util/meta.h"

#include "console.h"
#include "simavr/simavrdebugconsole.h"


using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

template<typename T>
concept bool isNonTerminal() {
    return requires(T v) {
        v.mChildren;
    };
}

struct A {
    constexpr A(uint8_t v = 0) : v(v) {}
    const uint8_t v = 0;
    void f(volatile uint8_t& x) const {
        x += v;
    }
};
struct B {
    constexpr B(uint8_t v = 0) : v(v) {}
    const uint8_t v = 1;
    void f(volatile uint8_t& x) const {
        x += v;
    }
};
struct C {
    constexpr C(uint8_t v = 0) : v(v) {}
    const uint8_t v = 2;
    void f(volatile uint8_t& x) const {
        x += v;
    }
};
struct D {
    constexpr D(uint8_t v = 0) : v(v) {}
    const uint8_t v = 3;
    void f(volatile uint8_t& x) const {
        x += v;
    }
};
struct E {
    constexpr E(uint8_t v = 0) : v(v) {}
    const uint8_t v = 4;
    void f(volatile uint8_t& x) const {
        x += v;
    }
};
template<typename... C>
struct Node {
    constexpr Node(const C&... c) : mChildren(c...) {}
    std::tuple<C...> mChildren;
};
template<typename T>
constexpr const T& first(const T& v) {
    return v; 
}
template<typename F, typename... T>
constexpr F first(const std::tuple<F, T...>& t) {
    return std::get<0>(t);
}
template<typename F, typename... T>
constexpr F first(const Node<F, T...>& n) {
    return first(n.mChildren);
}

namespace detail {
    template<typename F, typename... T, size_t... II>
    constexpr std::tuple<T...> rest(const std::tuple<F, T...>& t, std::index_sequence<II...>) {
        return std::tuple<T...>{std::get<II + 1>(t)...};
    }
}
template<typename T>
constexpr const T& rest(const T& v) {
    return v;
}
template<typename F, typename... T>
constexpr std::tuple<T...> rest(const std::tuple<F, T...>& t) {
    auto Indexes = std::make_index_sequence<sizeof...(T)>{};
    return detail::rest(t, Indexes);
}
template<typename F, typename... T>
std::tuple<T...> rest(const Node<F, T...>& n) {
    return rest(n.mChildren);
}

template<size_t N>
struct INode {
    std::array<uint8_t, N> mChildren {};
    void f(volatile uint8_t&) const {}
};

template<typename T>
constexpr const std::tuple<T> flat(const T& v, uint8_t&) {
    return std::tuple<T>(v);
}
template<typename...T>
constexpr auto flat(const std::tuple<T...>& t, uint8_t* c, uint8_t& p) {
    if constexpr(sizeof...(T) == 1) {
        auto t1 = flat(std::get<0>(t), p);
        *c = p++;
        return t1;
    }
    else {
        auto f = flat(first(t), p);
        *c = p++;
        auto r = flat(rest(t), (c + 1), p);
        return std::tuple_cat(f, r);
    }
}
template<typename...T>
constexpr auto flat(const Node<T...>& n, uint8_t& p = 0) {
    INode<std::tuple_size<decltype(n.mChildren)>::value> in;
    auto t1 = flat(n.mChildren, &in.mChildren[0], p);
    auto t2 = std::tuple(in);
    return std::tuple_cat(t1, t2);
}

volatile uint8_t sum = 0;

constexpr auto tree = Node(A(7), 
                           Node(C(6), 
                                D(5)
                                ), 
                           E(8), 
                           Node(A(3), 
                                B(4), 
                                Node(E(1), 
                                     E(2)
                                     )
                                ),
                           A(9),
                           A(10)
                           );

namespace detail {
    template<typename T>
    struct Info {
        static uint8_t children(const T&) {
            return 0;
        }
        static uint8_t child(const T&, uint8_t) {
            return 0;
        }
    };
    template<isNonTerminal T>
    struct Info<T> {
        static uint8_t children(const T& v) {
            return v.mChildren.size;
        }
        static uint8_t child(const T& v, uint8_t i) {
            return v.mChildren[i];
        }
    };
}

template<typename T>
constexpr uint8_t children(const T& tuple, uint8_t node) {
    return Meta::visitAt(tuple, node, [](const auto& item){
        return detail::Info<decltype(item)>::children(item);        
    });
}
template<typename T>
constexpr uint8_t child(const T& tuple, uint8_t node, uint8_t i) {
    return Meta::visitAt(tuple, node, [&](const auto& item){
        return detail::Info<decltype(item)>::child(item, i);
    });
}

int main() {
    
    while(true) {}    
}
