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

//const uint8_t SIZE = 64;

#define SIZE 64

struct Fifo {
    uint8_t array[SIZE];
    uint8_t in;
    uint8_t out;
};

volatile struct Fifo fifo = {};

bool empty(volatile struct Fifo* f) {
    return !(f->in ^ f->out);
}

bool empty1(volatile struct Fifo* f) {
    return (f->in == f->out);
}

int main()
{
//    fifo.in = (fifo.in + 1) % SIZE;
//    fifo.in = (fifo.in + 1) & (SIZE - 1);
    
    return empty1(&fifo);
}
