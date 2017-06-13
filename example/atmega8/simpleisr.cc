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

#include "config.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/util.h"
#include "mcu/ports.h"
#include "util/disable.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

#include <stdlib.h> // abort()

volatile uint8_t a0 = 0;
volatile uint8_t a1 = 0;
volatile uint8_t a2 = 0;

using timer1 = AVR::Timer8Bit<0>;
using timer2 = AVR::Timer16Bit<1>;
using timer3 = AVR::Timer8Bit<2>;

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using Pin0 = AVR::Pin<PortB, 0>;
using Pin1 = AVR::Pin<PortB, 1>;
using Pin2 = AVR::Pin<PortC, 0>;
using Pin3 = AVR::Pin<PortD, 0>;

struct Handler1 : public IsrBaseHandler<AVR::ISR::Timer<0>::Overflow> {
    static void isr() {
        ++a0;
    }
};
struct Handler2 : public IsrBaseHandler<AVR::ISR::Timer<1>::CompareA> {
    static void isr() {
        ++a1;
    }
};
struct Handler3 : public IsrBaseHandler<AVR::ISR::Timer<2>::Compare> {
    static void isr() {
        ++a2;
    }
};
struct Handler4 : public IsrBaseHandler<AVR::ISR::Timer<0>::Overflow> {
    static void isr() {
    }
};

using isrRegistrar = IsrRegistrar<Handler1, Handler2, Handler3>;

using terminal = SimAVRDebugConsole;
namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

int main() {
    Scoped<EnableInterrupt<>> interruptEnabler;
    
    isrRegistrar::init();
    
    constexpr auto t2 = AVR::Util::calculate<timer2>(1_Hz);
    timer2::prescale<t2.prescaler>();
    timer2::ocra<t2.ocr>();
    timer2::mode(AVR::TimerMode::CTC);

    while(true) {
        Util::delay(500_ms);
        std::cout << "A0: " << a0 << " A1: " << a1 << " A2: " << a2 << std::endl; 
    }    
}

ISR(TIMER0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::Overflow>();
}
ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIMER2_COMP_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::Compare>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
