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

#define USE_A1
#define USE_B

volatile uint8_t x_a = 0;

inline void A1_f() {
    x_a = 1;
}
inline void A2_f() {
    x_a = 2;
}

volatile uint8_t x_b = 0;
inline void B_f() {
    x_b = 3;
}

int main() {
#ifdef USE_A1
    A1_f();
#else
    A2_f();
#endif
    
#ifdef USE_B
    B_f();
#endif
    while(1) {}
}
