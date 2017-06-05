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

#include <stdint.h>

//#include "std/limits.h"
#include "std/traits.h"
//#include "std/concepts.h"
#include "std/array.h"
//#include "std/algorithm.h"
#include "util/fixedpoint.h"
#include "util/util.h"

int main() {
    Fraction<uint8_t> x{128};
    constexpr uint8_t base = 10;
    
    std::array<char, Util::numberOfDigits<decltype(x), base>()> data; // StringBuffer
    
    Util::ftoa(x, data);
    
    for(auto c: data) {
        if (c == '\0') {
            break;
        }
        GPIOR0 = c;
    }
    GPIOR0 = '\r';
    while(true) {}
}
