/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "mcu/avr/usart.h"
#include "mcu/ports.h"
#include "external/lcd.h"
#include "console.h"
#include "util/disable.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using LcdDB4 = AVR::Pin<PortC, 0>;
using LcdDB5 = AVR::Pin<PortC, 1>;
using LcdDB6 = AVR::Pin<PortC, 2>;
using LcdDB7 = AVR::Pin<PortC, 3>;

using LcdRS = AVR::Pin<PortC, 4>;
using LcdRW = AVR::Pin<PortC, 5>;
using LcdE  = AVR::Pin<PortB, 0>;

using LcdData = AVR::PinSet<LcdDB4, LcdDB5, LcdDB6, LcdDB7>;

using lcd = LcdHD44780<LcdData, LcdRS, LcdRW, LcdE>;

using terminal = AVR::Usart<0>;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

int main() {
    lcd::init();

    std::basic_ostream<lcd> lout;
    
    {
        Scoped<EnableInterrupt> ei;
        
        lout << "test" << std::endl;
        
        while(true) {}
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif

