/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

struct MCU {
    struct Timer {
        volatile uint8_t r1;
        template<int N>
        struct Address;
    };
};
template<>
struct MCU::Timer::Address<0> {
    static constexpr uintptr_t value = 0x25;
};

template<typename Component, int Number>
constexpr Component* getBaseAddr() {
    return reinterpret_cast<Component*>(Component::template Address<Number>::value);
}

struct Test {
    static constexpr auto a = getBaseAddr<MCU::Timer, 0>;
    static void foo() {
        a()->r1 = 42;
    }
};

int main() {
    
    if constexpr(true) {
        Test::foo();
    }
    while(true) {}
}
