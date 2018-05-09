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

#include "util/dassert.h"
#include "container/tree.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

struct A {
    constexpr A(uint8_t v = 0) : v(v) {}
    const uint8_t v = 0;
};
struct B {
    constexpr B(uint8_t v = 0) : v(v) {}
    const uint8_t v = 1;
};
struct C {
    constexpr C(uint8_t v = 0) : v(v) {}
    const uint8_t v = 2;
};
struct D {
    constexpr D(uint8_t v = 0) : v(v) {}
    const uint8_t v = 3;
};
struct E {
    constexpr E(uint8_t v = 0) : v(v) {}
    const uint8_t v = 4;
};
struct Menu {
    constexpr Menu(uint8_t v) 
        : mTitle(v) 
    {}
    const uint8_t mTitle;
};

constexpr auto tree = []{
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


int main() {
    uint8_t sum = 0;
    
    constexpr auto top = std::tuple_size<decltype(tree)>::value - 1;
    for(uint8_t n = 0; n < children(tree, top); ++n) {
//        std::outl<terminal>(n);
        Meta::visitAt(tree, child(tree, top, n), [&sum](const auto& item){
            typedef typename std::remove_reference<decltype(item)>::type ItemType;
            sum += F<std::remove_cv_t<ItemType>>::f(item);
            return 0;
        });
    }
    
    std::outl<terminal>(sum);
    return sum;
}
