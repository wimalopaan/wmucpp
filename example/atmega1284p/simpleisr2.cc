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

using timer1 = AVR::Timer8Bit<0>;

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using Pin0 = AVR::Pin<PortA, 0>;
using Pin1 = AVR::Pin<PortB, 0>;
using Pin2 = AVR::Pin<PortC, 0>;
using Pin3 = AVR::Pin<PortD, 0>;

struct Handler1 : public IsrBaseHandler<AVR::ISR::Timer<0>::CompareA> {
    static void isr() {
        Pin1::toggle();
    }
};

using isrRegistrar = IsrRegistrar<Handler1>;

using terminal = SimAVRDebugConsole;
namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

int main() {
    Scoped<EnableInterrupt<>> interruptEnabler;
    
    isrRegistrar::init();

    constexpr auto t1 = AVR::Util::calculate<timer1>(100_Hz);
    timer1::prescale<t1.prescaler>();
    timer1::ocra<t1.ocr>();
    timer1::mode(AVR::TimerMode::CTC);
    
    while(true) {
    }    
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}

