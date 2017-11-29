/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include <tuple>
#include <array>
#include <util/meta.h>
#include <util/tuple.h>

template<typename T, typename... C>
struct Node {
    typedef T node_type;
    constexpr Node(T&& n, C&&... c) : mData{std::forward<T>(n)}, mChildren(std::forward<C>(c)...) {}
    inline static constexpr auto size = sizeof...(C);
    T mData;
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
template<typename SizeType, typename Type>
struct INode {
    typedef Type type;
    typedef SizeType size_type;
    constexpr INode(const Type& d) : mData{d} {}
    Type mData;
    std::array<uint8_t, SizeType::value> mChildren {};
};

template<typename T> 
struct isINode : std::false_type {};
template<typename T, typename N>
struct isINode<INode<N, T>> : std::true_type {};

template<typename T>
constexpr bool isInode(const T&) {
    typedef typename std::remove_cv_t<typename std::remove_reference<T>::type> type;
    return isINode<type>::value;
}

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
template<typename N, typename... CC>
constexpr auto flat(const Node<N, CC...>& n, uint8_t& p = 0) {
    INode<std::integral_constant<size_t, n.size>, N> in{n.mData};
    auto t1 = flat(n.mChildren, &in.mChildren[0], p);
    auto t2 = std::tuple(in);
    return std::tuple_cat(t1, t2);
}

template<typename T>
constexpr auto make_tuple_of_tree(const T& tree) {
    uint8_t p = 0;
    return flat(tree, p);
}

namespace detail {
    template<typename T>
    concept bool isNonTerminal() {
        return requires(T& v) {
            v.mChildren;
        };
    }
    
    template<typename T>
    struct Info {
        static uint8_t children(const T&) {
            return 0;
        }
        static uint8_t child(const T&, uint8_t) {
            assert(false);
            return 0;
        }
    };
    template<isNonTerminal T>
    struct Info<T> {
        static uint8_t children(const T& v) {
            return std::tuple_size<decltype(v.mChildren)>::value;
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


template<typename T, auto... II>
struct IndexNode {
    typedef T type;
    typedef IndexNode index_type;
    T mData;
};

template<typename T>
struct is_indexnode : std::false_type {};
template<typename T, auto... II>
struct is_indexnode<IndexNode<T, II...>> : std::true_type {};

template<typename T>
constexpr bool isIndexNode(const T&) {
    typedef typename std::remove_cv_t<typename std::remove_reference<T>::type> type;
    return is_indexnode<type>::value;
}

template<typename T, auto... II>
constexpr uint8_t size(const IndexNode<T, II...>&) {
    return sizeof...(II);
}

template<typename T, auto... II>
constexpr auto children(const IndexNode<T, II...>&) {
    return std::array<uint8_t, sizeof...(II)>{II...};
}
