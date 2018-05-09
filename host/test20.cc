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

#include <iostream>
#include <cassert>
#include "container/tree.h"

struct A {
    constexpr A(int v = 0) : v(v) {}
    const int v = 0;
};
std::ostream& operator<<(std::ostream& out, const A& x) {
    return out << "A[" << x.v << "]";
}
struct B {
    constexpr B(int v = 0) : v(v) {}
    const int v = 1;
};
std::ostream& operator<<(std::ostream& out, const B& x) {
    return out << "B[" << x.v << "]";
}
struct C {
    constexpr C(int v = 0) : v(v) {}
    const int v = 2;
};
std::ostream& operator<<(std::ostream& out, const C& x) {
    return out << "C[" << x.v << "]";
}
struct D {
    constexpr D(int v = 0) : v(v) {}
    const int v = 3;
};
std::ostream& operator<<(std::ostream& out, const D& x) {
    return out << "D[" << x.v << "]";
}
struct E {
    constexpr E(int v = 0) : v(v) {}
    const int v = 4;
};
std::ostream& operator<<(std::ostream& out, const E& x) {
    return out << "E[" << x.v << "]";
}

template<::detail::isNonTerminal T>
std::ostream& operator<<(std::ostream& out, const T& n) {
    out << "IN[";
    for(const auto& v : n.mChildren) {
        if (&v != &n.mChildren[0]) {
            out << ',';
        } 
        out << (int)v;
    }
    return out << "]";
}


constexpr auto tree = []{
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
    uint8_t p = 0;
    return flat(tree, p);
}();

template<typename ItemType>
struct F {
    static int f(const ItemType& item) {
        return item.v;
    }
};
template<typename N, typename T>
struct F<INode<N, T>> {
    static int f(const INode<N, T>&) {
        return 0;
    }
};

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

int main() {
    std::cout << tree << '\n';    
    
    uint8_t sum = 0;
    
    constexpr auto top = std::tuple_size<decltype(tree)>::value - 1;
    for(uint8_t n = 0; n < children(tree, top); ++n) {
        Meta::visitAt(tree, child(tree, top, n), [&sum](const auto& item){
            std::cout << item << '\n';
            typedef typename std::remove_reference<decltype(item)>::type ItemType;
            sum += F<std::remove_cv_t<ItemType>>::f(item);
            return 0;
        });
    }
    std::cout << "sum:" << (int)sum << '\n';
}
