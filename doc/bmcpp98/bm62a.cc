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
#include "std/array.h"
#include "std/initializer_list.h"
#include "std/algorithm.h"

#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

volatile uint8_t x;

struct IX {
    virtual void f(volatile uint8_t& x) const = 0;
    virtual uint8_t size() const = 0;
    virtual IX* child(uint8_t index) const = 0;
//    virtual ~IX() = default; // need delete
};

struct AT : public IX {
    virtual uint8_t size() const {return 0;}
    virtual IX* child(uint8_t) const {return nullptr;}
};

struct A : public AT {
    constexpr A(uint8_t v = 0) : v(v) {}
    const uint8_t v = 0;
    void f(volatile uint8_t& x) const override {
        x += v;
    }
};

struct B : public AT {
    constexpr B(uint8_t v = 0) : v(v) {}
    const uint8_t v = 1;
    void f(volatile uint8_t& x) const override {
        x += v;
    }
};

struct C : public AT {
    constexpr C(uint8_t v = 0) : v(v) {}
    const uint8_t v = 2;
    void f(volatile uint8_t& x) const override {
        x += v;
    }
};
struct D : public AT {
    constexpr D(uint8_t v = 0) : v(v) {}
    const uint8_t v = 3;
    void f(volatile uint8_t& x) const override {
        x += v;
    }
};
struct E : public AT {
    constexpr E(uint8_t v = 0) : v(v) {}
    const uint8_t v = 4;
    void f(volatile uint8_t& x) const override {
        x += v;
    }
};
struct M : public IX {
    template<typename...T>
    constexpr M(T*... ps) : mChildren{ps...}, mSize(sizeof...(T)) {}
    virtual uint8_t size() const {return mSize;}
    virtual IX* child(uint8_t index) const {return mChildren[index];}
    
    void f(volatile uint8_t&) const override {}

    const std::array<IX*, 8> mChildren{};
    const uint8_t mSize = 0;
};

volatile uint8_t index = 2;

E e1(1);
E e2(2);
M m1(&e1, &e2);

A a1(3);
B b1(4);
M m2(&a1, &b1, &m1);

C c1(5);
D d1(6);
M m3(&c1, &d1);

A a2(7);
E e3(8);

A a3(9);
A a4(10);
    
constexpr M m4(&a2, &m3, &e3, &m2, &a3, &a4);

volatile uint8_t sum;

int main() {
    for(uint8_t i = 0; i < m4.size(); ++i) {
//        std::outl<terminal>(i);
        if (auto cs = m4.child(i)) {
                cs->f(sum);
        }
    }  
//    std::outl<terminal>(sum);
    while(true) {}
}