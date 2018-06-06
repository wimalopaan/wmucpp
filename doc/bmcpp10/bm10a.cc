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

#define LAMBDA

#include <stdint.h>

#include <limits>
#include <array>
#include "util/util.h"

template<typename C, typename L>
void visit(const C& data, const L& f) {
    [&]<auto... II>(std::index_sequence<II...> ){
        (f(data[II]), ...);
    }(std::make_index_sequence<data.size>{});
}

int main() {
    constexpr uint8_t x = 42;
    constexpr uint8_t base = 10;
    constexpr auto data = []<typename T>(T v){
            std::array<char, Util::numberOfDigits<T, base>() + 1> data; // StringBuffer
            Util::itoa<base>(v, data);
            return data;
    }(x);

#ifdef LAMBDA    
    visit(data, [](char v){
        if (v != '\0') {
            GPIOR0 = v;
        }
    });
#else
    for(auto c: data) {
        if (c == '\0') {
            break;
        }
        GPIOR0 = c;
    }
#endif
    GPIOR0 = '\r';
    while(true) {}
}

