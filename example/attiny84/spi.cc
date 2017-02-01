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

// sudo avrdude -p attiny84 -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usi.h"
#include "hal/softspimaster.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using led = AVR::Pin<PortB, 0>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using UsiSS = AVR::Pin<PortA, 3>;
using Usi = AVR::Usi<0, UsiSS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

int main() 
{
    terminal::init();
    
    led::dir<AVR::Output>();
    led::high();
    
    Usi::init();

    std::cout << "attiny usi test"_pgm << std::endl;
    
    while(true) {
        led::toggle();
        if (Usi::select()) {
            uint8_t counter = Usi::get(); // number
            for(uint8_t i = 0; i < counter; ++i) {
                Usi::get();
            }
        }
    }    
}
