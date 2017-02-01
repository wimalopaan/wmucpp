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
typedef uint32_t size_t;

uint8_t foo(size_t v) {
    return v + 1;
}

int main()
{
    const int Size = 10;
    uint8_t values[Size];
    for(uint8_t l = 0; l < Size; ++l) {
        values[l] = 0;
    }
    values[0] = 1;
    values[1] = 1;
    values[2] = 1;
    
    const int NumberOfIndices = 3;
    int indices[NumberOfIndices];
    for(uint8_t l = 0; l < NumberOfIndices; ++l) {
        indices[l] = foo(l);
    }
    
    uint8_t sum = 0;
    for(uint8_t l = 0; l < NumberOfIndices; ++l) {
        sum += values[indices[l]];
    }
    return sum;    
}
