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

volatile uint8_t global = 0;

struct Array {
    uint8_t* ptr;
    const uint8_t size;
};

void foo(const struct Array a) {
    for(const uint8_t* p = a.ptr; p != a.ptr + a.size; ++p) {
        global += *p;
    }
}

int main()
{
    uint8_t a[10] = {1, 2};

    struct Array ax = {a, sizeof(a)};

    foo(ax);

    while(true);
}
