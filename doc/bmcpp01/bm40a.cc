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

//#include <avr/pgmspace.h>

const int x __attribute__((address_space(1))) = 3;
const int y = x;

//char* text = "abc";

const char c = "abc"[0];

//const char date[] PROGMEM = {__DATE__[0], __DATE__[1]};
const char date[] __attribute__((address_space(1))) = {__DATE__[0], __DATE__[1]};
const char bla[] __attribute__((address_space(1))) = "0123456789abcdefghijklmnopqrstuvwABCDEFGHIJKLMNOPQRSTUVW";

volatile char cc;

int main() {
//    cc = date[0];
    
    while(bla[cc]) {
        cc = bla[cc];
    }
    
    return cc;
}
