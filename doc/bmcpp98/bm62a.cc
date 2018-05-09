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

#include <array>
#include <initializer_list>
#include <algorithm>

#include "util/dassert.h"

#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

struct IX {
    constexpr IX(uint8_t v = 0) : v(v) {}
    virtual uint8_t f() const {return v;}
    virtual uint8_t size() const = 0;
    virtual IX* child(uint8_t index) const = 0;
//    virtual ~IX() = default; // need delete
    const uint8_t v = 0;
};

struct AT : public IX {
    constexpr AT(uint8_t v) : IX(v) {}
    virtual inline uint8_t size() const {return 0;}
    virtual inline IX* child(uint8_t) const {return nullptr;}
};

struct A final : public AT {
    constexpr A(uint8_t v = 0) : AT(v) {}
};

struct B final : public AT {
    constexpr B(uint8_t v = 0) : AT(v) {}
};

struct C final : public AT {
    constexpr C(uint8_t v = 0) : AT(v) {}
};
struct D final : public AT {
    constexpr D(uint8_t v = 0) : AT(v) {}
};
struct E final : public AT {
    constexpr E(uint8_t v = 0) : AT(v) {}
};

template<uint8_t N = 8> 
struct M final : public IX {
    template<typename...T>
    constexpr inline M(T*... ps) : mChildren{ps...} {
        static_assert(N >= sizeof...(T));
    }
    virtual inline uint8_t size() const {return N;}
    virtual inline IX* child(uint8_t index) const {return mChildren[index];}
private:
    const std::array<IX*, N> mChildren{};
    static inline constexpr uint8_t mSize = N;
};

template<typename... MM>
constexpr M<sizeof...(MM)> make_M(MM*... mm) {
    return M<sizeof...(MM)>(mm...);
}

E e1(1);
E e2(2);
auto m1 = make_M(&e1, &e2);

A a1(3);
B b1(4);
auto m2 = make_M(&a1, &b1, &m1);

C c1(5);
D d1(6);
auto m3 = make_M(&c1, &d1);

A a2(7);
E e3(8);

A a3(9);
A a4(10);
    
auto m4 = make_M(&a2, &m3, &e3, &m2, &a3, &a4);

int main() {
    uint8_t sum = 0;
    for(uint8_t i = 0; i < m4.size(); ++i) {
//        std::outl<terminal>(i);
        if (auto cs = m4.child(i)) {
                sum += cs->f();
        }
    }  
    return sum;
//    std::outl<terminal>(sum);
    while(true) {}
}
