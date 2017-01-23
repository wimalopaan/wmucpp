/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

volatile uint8_t x = 0;
volatile uint8_t y = 0;

inline void setFlag(volatile uint8_t& f) {
    f = 0x01;
}

inline bool testAndClear(volatile uint8_t& f) {
    f &= 0x0f;
    f = __builtin_avr_swap(f);
    return f & 0xf0;
}

int main()
{
    setFlag(x);
    if (testAndClear(x)) {
        y = 1;
    }
    while(true);
}
