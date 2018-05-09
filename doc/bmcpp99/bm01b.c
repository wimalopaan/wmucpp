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
#include <avr/interrupt.h>

volatile uint8_t x = 0;

int main()
{
    sei();
    TCCR0B |= _BV(CS01) | _BV(CS11);
    OCR0A = 195;
    TIMSK0 |= 13;
    TCCR0A = 12;
    while(1) {}
}

ISR(TIMER0_COMPA_vect) {
    ++x;
}

ISR(TIMER0_COMPB_vect) {
    ++x;
}
