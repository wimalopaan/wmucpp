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
#include "std/type_traits"
#include "mcu/concepts.h"

volatile uint8_t x = 0;

template<typename T>
void foo(const T& v) {
    x = 2 * v;    
}

template<MCU::RegisterType T>
void foo(T v) {
    x = v;    
}

int main() {
    uint8_t y = 2;
    foo(y);

    uint16_t z = 3;
    foo(z);
    
    while(true) {}
}
