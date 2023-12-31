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
#include <stdbool.h>

volatile uint8_t global; //-
volatile bool b = false;
volatile bool c = false;

//[pointer
bool fooA(uint8_t* x) {
    *x = global; //-
    return b;
}
bool fooB(uint8_t* x, uint8_t y) {
    global = *x + y; //-
    *x = global;
    return b;
}
//]
//[main
int main()
{
    uint8_t a = 42;
    uint8_t b = 43;

    c = fooA(&a);
    c = b && fooB(&a, b);

    while(true);
}
//]
