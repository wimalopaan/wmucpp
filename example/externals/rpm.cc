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

#include "mcu/ports.h"
#include "external/rpm.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/isr.h"
#include "hal/softspimaster.h"
#include "util/disable.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using led = AVR::Pin<PortB, 4>;

using rpmTimer = AVR::Timer8Bit<0>;
//using rpmTimer = AVR::Timer16Bit<1>;

using rpm = RpmFromAnalogComparator<0, rpmTimer>;

using isrRegistrar = IsrRegistrar<rpm>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

int main() {
    isrRegistrar::init();
    rpm::init();

    led::template dir<AVR::Output>();
    led::off();
    
    {
        Scoped<EnableInterrupt> ei;        
        std::cout << "RPM with AComparator example"_pgm << std::endl;
        
        while(true) {
            std::cout << "Period: "_pgm << rpm::period() << std::endl;
//            std::cout << "frequency: "_pgm << rpm::frequency() << std::endl;
        }
    }
}

ISR(ANALOG_COMP_vect) {
    led::toggle();
    isrRegistrar::isr<AVR::ISR::AdComparator<0>::Edge>();
}

#ifndef NDEBUG
void assertFunction(const char*, const char*, const char*, unsigned int) {
    while(true) {};
}
#endif
