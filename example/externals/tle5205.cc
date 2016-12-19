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

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "external/tle5205.h"
#include "hal/softspimaster.h"
#include "util/disable.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using Tle5205In1 = AVR::Pin<PortB, 0>;
using Tle5205In2 = AVR::Pin<PortB, 1>;
using Tle5205Error = AVR::Pin<PortB, 2>;

using tleTimer = AVR::Timer8Bit<0>;

using hbridge = TLE5205<Tle5205In1, Tle5205In1, Tle5205Error, tleTimer>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

static constexpr std::hertz pwmFrequency = 1000_Hz;

int main() {
    terminal::init();
    
    hbridge::init<pwmFrequency>();
    
    {
        Scoped<EnableInterrupt> ei;
        std::cout << "tle5205 example"_pgm << std::endl;
        
        constexpr auto prescaler = AVR::Util::prescalerForAbove<tleTimer>(pwmFrequency);
        std::cout << "prescaler: "_pgm << prescaler << std::endl;
        
        while(true) {}
    }    
}

#ifndef NDEBUG
void assertFunction(bool b, const char*, const char*, unsigned int) {
   if (!b) {
       while(true) {
       }
   }
}
#endif
