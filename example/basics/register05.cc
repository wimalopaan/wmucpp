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

struct Component {
    typedef uint8_t value_type;
    union {
        volatile value_type a;
        volatile value_type b;
    };
};

volatile Component c1;
volatile uint8_t g = 0;

int main() {
    // das geht zwar mit g++
    c1.a = 1;
    g = c1.b;    

    c1.b = 2; // das Beschreiben ist aber UB
    
    while(true) {}
}
