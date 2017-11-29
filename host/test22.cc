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

#include <iostream>
#include <cassert>
#include "container/tree.h"

namespace std {
    namespace detail {
        template<auto... II, typename... T>
        std::ostream& print_tuple(std::ostream& out, std::index_sequence<II...>, const std::tuple<T...>& tuple) {
            ((out << std::get<II>(tuple) << " "),...);
            return out;
        }
    }
    template<typename... T>
    std::ostream& operator<<(std::ostream& out, const std::tuple<T...>& tuple) {
        return detail::print_tuple(out, std::make_index_sequence<sizeof...(T)>{}, tuple);  
    }
}

template<typename T>
void ptest(const T&) {
    T::_;
}


template<typename D>
struct MenuItem {
    const int v = 0;
};

struct A : MenuItem<A> {
    constexpr A(int v = 0) : MenuItem<A>{v} {}
};
std::ostream& operator<<(std::ostream& out, const A& x) {
    return out << "A[" << x.v << "]";
}
struct B : MenuItem<B> {
    constexpr B(int v = 0) : MenuItem<B>{v} {}
};
std::ostream& operator<<(std::ostream& out, const B& x) {
    return out << "B[" << x.v << "]";
}
struct C : MenuItem<C> {
    constexpr C(int v = 0) : MenuItem<C>{v} {}
};
std::ostream& operator<<(std::ostream& out, const C& x) {
    return out << "C[" << x.v << "]";
}
struct D : MenuItem<D> {
    constexpr D(int v = 0) : MenuItem<D>{v} {}
};
std::ostream& operator<<(std::ostream& out, const D& x) {
    return out << "D[" << x.v << "]";
}
struct E : MenuItem<E> {
    constexpr E(int v = 0) : MenuItem<E>{v} {}
};
std::ostream& operator<<(std::ostream& out, const E& x) {
    return out << "E[" << x.v << "]";
}

struct Menu {
    constexpr Menu(int v) : mTitle(v) {}
    const int mTitle;
};

std::ostream& operator<<(std::ostream& out, const Menu& m) {
    return out << "Menu[" << m.mTitle << "]";
}

template<::detail::isNonTerminal T>
std::ostream& operator<<(std::ostream& out, const T& n) {
    out << "IN[";
    for(const auto& v : n.mChildren) {
        out << (int)v << ',';
    }
    out << n.mData;
    return out << "]";
}

template<typename T, auto... II>
struct I {
    typedef I index_type;
    inline static constexpr uint8_t size = sizeof...(II);
    inline static constexpr uint8_t indexes[] = {II...};
};

template<typename T>
requires requires() {
    typename T::index_type;
}
std::ostream& operator<<(std::ostream& out, const T& i) {
    out << "I[";
    for(uint8_t n = 0; n < i.size; ++n) {
        out << (int)i.indexes[n] << ' ';
    }
    return out <<  "]";
}


constexpr auto tree2 = []{
    constexpr auto tree = Node(Menu(1), 
                               A(7), 
                               Node(Menu(2),
                                    B(6), 
                                    C(5)
                                    ));

    constexpr auto ftree = make_tuple_of_tree(tree);

    
    
    return ftree;
}();

int main() {
    std::cout << tree2 << '\n';    
//    ptest(tree2);
}
