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

#include <cassert>
#include "container/tree.h"
#include "util/concepts.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

struct A {
    constexpr A(uint8_t v = 0) : av(v) {}
    constexpr auto v() const {
        return av;
    }
    const uint8_t av = 0;
};
struct B {
    constexpr B(uint8_t v = 0) : bv(v) {}
    constexpr auto v() const {
        return bv;
    }
    const uint8_t bv = 1;
};
struct C {
    constexpr C(uint8_t v = 0) : cv(v) {}
    constexpr auto v() const {
        return cv;
    }
    const uint8_t cv = 2;
};
struct D {
    constexpr D(uint8_t v = 0) : dv(v) {}
    constexpr auto v() const {
        return dv;
    }
    const uint8_t dv = 3;
};
struct E {
    constexpr E(uint8_t v = 0) : ev(v) {}
    constexpr auto v() const {
        return ev;
    }
    const uint8_t ev = 4;
};
struct Menu {
    constexpr Menu(uint8_t v) 
        : mTitle(v) 
    {}
    const uint8_t mTitle;
};

constexpr auto flat_tree = []{
    constexpr auto tree = Node(Menu(1), 
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
                               );
    constexpr auto ftree = make_tuple_of_tree(tree);
    return ftree;
}();

template<auto... II, Util::Callable L>
constexpr auto inode_to_indexnode(std::index_sequence<II...>, const L& callable) {
    constexpr auto inode = callable();
    static_assert(isInode(inode), "use a callable returning an INode<>");
    typedef typename decltype(inode)::type dataType;
    return IndexNode<dataType, Index<inode.mNumber>, ParentIndex<inode.mParent>, inode.mChildren[II]...>{inode.mData};
}


template<Util::Callable L>
constexpr auto transform(const L& callable) {
    constexpr auto tuple = callable();
    static_assert(Util::isTuple(tuple), "use constexpr callabe returning a tuple");
    
    if constexpr(Util::size(tuple) == 0) {
        return std::tuple<>();
    }
    else {
        constexpr auto first = std::get<0>(tuple);    
        /*constexpr */auto rest = [&]{return Util::tuple_tail(tuple);};
        
        if constexpr(isInode(first)) {
            constexpr auto indexnode = inode_to_indexnode(std::make_index_sequence<first.mChildren.size>{}, [&]{return first;});
            return std::tuple_cat(std::tuple(indexnode), transform(rest));        
        }
        else {
            return std::tuple_cat(std::tuple(first), transform(rest));
        }
    }
}

auto t2 = transform([&]{return flat_tree;}); 

template<typename T>
constexpr uint8_t f(const T& v) {
    return v.v();
}
template<typename N, typename Index, typename Parent, auto... II>
constexpr uint8_t f(const IndexNode<N, Index, Parent, II...>&) {
    return 0;
}

int main() {
    uint8_t sum = 0;
    constexpr uint8_t topIndex = std::tuple_size<decltype(t2)>::value - 1;
    for(auto n : children(std::get<topIndex>(t2))) {
        Meta::visitAt(t2, n, [&sum](const auto& item){
            if constexpr(!isIndexNode(item)) {
                sum += item.v();
            }
            return 0;
        });
    }
    std::outl<terminal>(sum);
    return sum;
}
