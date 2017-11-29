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

#include <tuple>
#include "util/dassert.h"
#include "util/meta.h"
#include "container/tree.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

struct A {
    constexpr A(uint8_t v = 0) : v(v) {}
    const uint8_t  v = 0;
};
struct B {
    constexpr B(uint8_t v = 0) : v(v) {}
    const uint8_t v = 1;
};
struct C {
    constexpr C(uint8_t v = 0) : v(v) {}
    const uint8_t v = 2;
};


namespace std {
    template<typename S, typename T>
    requires requires() {
        typename T::index_type;
    }
    void outl(const T& i) {
        std::out<S>("I["_pgm);
        for(uint8_t n = 0; n < i.size(); ++n) {
            std::out<S>(i.get(n));
        }
        std::outl<S>("]"_pgm);
    }
    template<typename S>
    void outl(const A&) {
        std::outl<terminal>("A"_pgm);    
    }
    template<typename S>
    void outl(const B&) {
        std::outl<terminal>("B"_pgm);    
        
    }
    template<typename S>
    void outl(const C&) {
        std::outl<terminal>("C"_pgm);    
        
    }
}

auto tree = std::tuple(A(), B(), IndexNode<C, 0, 1>());

int main() {
//    std::outl<terminal>("Bla"_pgm);    
    
    //        ptest(tree);
    
    constexpr auto size = std::tuple_size<decltype(tree)>::value;
    for(uint8_t n = 0; n < size; ++n) {
        Meta::visitAt(tree, n, [](const auto& item){
//            std::outl<terminal>(item);
            return 0;
        });
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    //    while(true) {}
}
#endif

