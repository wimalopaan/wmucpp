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

//#define NDEBUG

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/util.h"
#include "util/disable.h"

volatile uint8_t x = 0;

using timer1 = AVR::Timer8Bit<0>;

struct Handler1 final : public IsrBaseHandler<AVR::ISR::Timer<0>::CompareA> {
    static void isr() asm("handler1"){
        ++x;
    }
};

struct Handler2 final : public IsrBaseHandler<AVR::ISR::Timer<0>::CompareB> {
    static void isr() asm("handler2"){
        --x; // ist diese ISR gleich zu Handler1, so wird sie aufgerufen -> das verursacht erhöhten Aufwand durch push/pop Sequenzen
    }
};

using isrRegistrar = IsrRegistrar<Handler1, Handler2>;

int main()
{
    Scoped<EnableInterrupt<>> interruptEnabler;
    
    isrRegistrar::init();
    
    constexpr auto t1 = AVR::Util::calculate<timer1>(100_Hz);
    timer1::prescale<t1.prescaler>();
    timer1::ocra<t1.ocr>();
    timer1::mode(AVR::TimerMode::CTC);
    
    while(true) {
        Handler1::isr();
    }
}

ISR(TIMER0_COMPA_vect) asm("isr1");
ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>(); // hier findet sogar inlining statt
}

ISR(TIMER0_COMPB_vect) asm("isr2");
ISR(TIMER0_COMPB_vect){
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareB>(); // auch hier inlining
}
