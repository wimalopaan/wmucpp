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

#include <stdint.h>

#include <limits>
#include <array>
#include <type_traits>
#include <algorithm>
#include "std/concepts.h"
#include "util/util.h"

int main() {
    uint8_t x = 42;
    constexpr uint8_t base = 10;
    std::array<char, Util::numberOfDigits<decltype(x), base>() + 1> data; // StringBuffer
    
    Util::itoa<base>(x, data);
    
    for(auto c: data) {
        if (c == '\0') {
            break;
        }
        GPIOR0 = c;
    }
    GPIOR0 = '\r';
    while(true) {}
}

