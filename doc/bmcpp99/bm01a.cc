/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/avr8.h"
#include "mcu/avr/isr.h"

volatile uint8_t x = 0;

struct Handler1 final : public IsrBaseHandler<AVR::ISR::Int<0>>
{
    static void isr() {
        ++x;
    }
};

struct Handler2 final : public IsrBaseHandler<AVR::ISR::Int<1>>
{
    static void isr() {
        ++x;
    }
};

using isrRegistrar = IsrRegistrar<Handler1, Handler2>;

int main()
{
    isrRegistrar::init();    
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}

ISR(TIMER0_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareB>();
}
