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

volatile uint8_t global; //-
volatile bool b = false;
volatile bool c = false;

//[value
bool foo(uint8_t x) {
    global = x; //-
    return b;
}

bool foo(uint8_t x, uint8_t y) {
    global = x + y; //-
    return b;
}
//]
//[main
int main()
{
    uint8_t a = 42;
    uint8_t b = 43;

    c = foo(a);
    c = b && foo(a, b);

    while(true);
}
//]
