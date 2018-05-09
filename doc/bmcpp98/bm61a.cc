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
#include "std/tuple"

#include "util/meta.h"

volatile uint8_t x;

struct A {
    const uint8_t v = 0;
    void f() const {
        x = v;
    }
    void g() {
        x = v;
    }
};

struct B {
    const uint8_t v = 1;
    void f() const {
        x = v;
    }
    void g() {
        x = v;
    }
};

struct C {
    const uint8_t v = 2;
    void f() const {
        x = v;
    }
    void g() {
        x = v;
    }
};

volatile uint8_t index = 2;

int main() {
    const std::tuple<A, B, C> t;
    
//    Meta::visitAt(t, index, [](const auto& v){v.f();});
 
    while(true) {}
}
