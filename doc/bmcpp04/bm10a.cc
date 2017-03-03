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
#include "std/array.h"

volatile uint8_t x = 0;
volatile uint8_t y = 0;

constexpr uint16_t size = 100;
constexpr uint16_t offset = 1; // wenn offset == 0, kommt alles in data segment, andernfalls auf den Stack

int main() {
    constexpr const auto array = [](){
        std::array<uint8_t, size + offset> a;
        for(std::remove_cv_t<decltype(size)> i = 0; i < size; ++i) {
            a[i] = i + 1;
        }
        return a;
    }();
    
    y = array[x];
    
    while(true) {}
}


#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView& , unsigned int) {
    while(true) {}
}
#endif

