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

#include <stdint.h>

__flash const char s[]  = "abcdef";
__flash const char s1[] = "Bla abcdef";

volatile uint8_t x;

volatile uint8_t v1;
volatile uint8_t v2;

int main() {
    for(uint8_t i = 0; s[i] != '\0'; ++i) {
        x += s[i] + v1;    
    }
    for(uint8_t i = 0; s1[i] != '\0'; ++i) {
        x += s1[i] + v2;    
    }
}