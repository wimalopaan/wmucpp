/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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
#include "std/traits.h"

void f() {
//    throw 42; // undefined ref to cxa_...
}

volatile uint8_t x;

int main() {
#define LOOPS (10)
    for(int loop = 0; loop < LOOPS; loop++) {
        x = 0;
    }
    
//    constexpr auto LOOPS = 142;    
    
//    for(std::remove_cv_t<decltype(LOOPS)> loop = 0; loop < LOOPS; loop++) {
//        x = 0;
//    }
    
    //    f();    
}

