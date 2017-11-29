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

constexpr auto tree = make_tuple_of_tree(Node(Menu(1),
                                              A(7),
                                              Node(Menu(2),
                                                   C(6), 
                                                   D(5)
                                                   ), 
                                              E(8), 
                                              Node(Menu(3),
                                                   A(3), 
                                                   B(4), 
                                                   Node(Menu(4),
                                                        E(1), 
                                                        E(2)
                                                        )
                                                   ),
                                              A(9),
                                              A(10)
                                              )
                                         );

//constexpr auto tree = []{
//    auto tree = Node(Menu(1),
//                     A(7),
//                     Node(Menu(2),
//                          C(6), 
//                          D(5)
//                          ), 
//                     E(8), 
//                     Node(Menu(3),
//                          A(3), 
//                          B(4), 
//                          Node(Menu(4),
//                               E(1), 
//                               E(2)
//                               )
//                          ),
//                     A(9),
//                     A(10)
//                     );
//    uint8_t p = 0;
//    return flat(tree, p);
//}();

//constexpr auto tree = []{
//    constexpr auto tree = Node(Menu(1),
//                              A(7), B(3)
//                               );
//    uint8_t p = 0;
//    return flat(tree, p);
//}();

template<typename ItemType>
struct F {
    static int f(const ItemType& item) {
        return item.v;
    }
};
template<>
struct F<Menu> {
    static int f(const Menu&) {
        return 0;
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

template<typename T>
void ptest(const T&) {
    T::_;
}

int main() {
    std::cout << tree << '\n';    
    
    //    ptest(tree);
    
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
