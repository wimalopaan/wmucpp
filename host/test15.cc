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
#include <tuple>
//#include "util/meta.h"

//template<size_t Index, typename Tuple>
//struct Entry {
//    Tuple tuple;
//    inline static constexpr size_t index = Index;
//};

//template<typename NewType, typename... T, typename... Children>
//constexpr auto add(const std::tuple<T...>& t = std::tuple<>{}, Children... c) {
//    auto nextTuple = std::tuple_cat(t, std::tuple(INode(NewType(), c...)));
//    return Entry<sizeof...(T), decltype(nextTuple)>{nextTuple};
//}


struct A {
    void f() const {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
};
struct B {
    void f() const {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
};
struct C {
    void f() const {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
};
struct D {
    void f() const {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
};
struct E {
    void f() const {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
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
    void f() const {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        for(const auto& i: mChildren) {
            std::cout << i << std::endl;
        }
    }
    std::array<size_t, N> mChildren {};
};

template<typename T>
constexpr const std::tuple<T> flat(const T& v, size_t&) {
    return std::tuple<T>(v);
}
template<typename...T>
constexpr auto flat(const std::tuple<T...>& t, size_t* c, size_t& p) {
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
constexpr auto flat(const Node<T...>& n, size_t& p = 0) {
    INode<std::tuple_size<decltype(n.mChildren)>::value> in;
    auto t1 = flat(n.mChildren, &in.mChildren[0], p);
    auto t2 = std::tuple(in);
    return std::tuple_cat(t1, t2);
}

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

//template<typename T>
//constexpr uint8_t children(const T& tuple, uint8_t node) {
//    return Meta::visitAt(tuple, node, [](const auto& item){
//        return detail::Info<decltype(item)>::children(item);        
//    });
//}
//template<typename T>
//constexpr uint8_t child(const T& tuple, uint8_t node, uint8_t i) {
//    return Meta::visitAt(tuple, node, [&](const auto& item){
//        return detail::Info<decltype(item)>::child(item, i);
//    });
//}

int main() {
    constexpr auto fl2 = [](){
        constexpr auto tree = Node(A(), 
                                   Node(C(), D()), 
                                   E(), 
                                   Node(A(), 
                                        B(), 
                                        Node(E(), 
                                             E())
                                        )
                                   );
        size_t p = 0;
        return flat(tree, p);
    }();
    
   
    std::cout << "Size: " << std::tuple_size<decltype(fl2)>::value << '\n';
            
//    for(uint8_t n = 0; n < children(fl, top); ++n) {
        
//    }    
    
    for(uint8_t i = 0; i < std::tuple_size<decltype(fl2)>::value; ++i) {
        Meta::visitAt(fl2, i, [](const auto& v) {v.f();});
    }    
    
}
