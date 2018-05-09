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

#include <stdint.h>

static volatile uint8_t x = 0;

struct A {
    const uint8_t length = 0;
    const uint8_t startPosition = 0;
};
template<int N>
struct B {
    static constexpr A Es{1, 1};
    static constexpr A Ist{2, 1}; // Ist
    static constexpr A Uhr{3, 1}; // Uhr
    
    static void test() {
        foo(Es);
        foo(Ist);
        foo(Uhr);
    }
private:
    static void foo(const A& w){
        for(uint8_t i = 0; i < w.length; ++i) {
            x = w.startPosition + w.length + i;
//            x = w.startPosition  + i;
        }
    }

};

int main() {
    using w = B<0>;
    w::test();        
    while(true);
}
