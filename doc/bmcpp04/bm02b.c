/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <avr/io.h>

volatile uint8_t a = 0;
volatile uint8_t b = 0;

volatile uint8_t x = 1;

void initB() {
    a = 1;
    GPIOR0 = 'b';
}

void initA() {
    b = 2;
    GPIOR0 = 'a';
}

int main()
{
    const uint8_t a = 1;
    const uint8_t b = 1;

    if (x == a) {
        initA();
    }
    if (x == b) {
        initB();
    }
    GPIOR0 = '\r';
    GPIOR0 = '\n';

    while(true);
}
