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

#include <stdint.h>
#include <stdbool.h>

volatile uint8_t global; //-

//[pointer
void fooA(volatile uint8_t* x) {
    *x = global; //-
}
void fooB(volatile uint8_t* x, volatile uint8_t* y) {
    *x = global; //-
    *y = global; //-
}
//]
//[main
int main()
{
    volatile uint8_t a = 42;
    volatile uint8_t b = 43;

    fooA(&a);
    fooB(&a, &b);

    while(true);
}
//]
