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

#include<stdint.h>

__flash const char  ok[] = "ok";
__flash const char nok[] = "nok";

volatile uint8_t x;

volatile uint8_t v1;
volatile uint8_t v2;

void f1(__flash const char* s) {
    for(uint8_t i = 0; s[i] != '\0'; ++i) {
        x += s[i] + v1;    
    }
}
void f2(__flash const char* s) {
    for(uint8_t i = 0; s[i] != '\0'; ++i) {
        x += s[i] + v2;    
    }
}

int main() {
    f1(ok);    
    f2(nok);    
    f1(nok);    
    f2(ok);    
}
