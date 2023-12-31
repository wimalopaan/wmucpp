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

#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"
#include "util/memory.h"

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

using LcdData = AVR::PinSet<AVR::UsePgmTable, LcdDB4, LcdDB5, LcdDB6, LcdDB7>;

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

namespace std {
    constexpr terminal cout;
    constexpr std::lineTerminator<CRLF> endl;
}

//volatile uintN_t<LcdData::size> x{5};
uintN_t<LcdData::size> x{5};


int main()
{
    std::cout << "start"_pgm << std::endl;   

    LcdData::allOff();

    LcdData::set(uintN_t<LcdData::size>{13});
    
    LcdData::set<13>();
    LcdData::set<1>();
    LcdData::set<5>();

    std::cout << std::to_integer<uint8_t>(PortC::get()) << std::endl;    

    LcdRS::on();
    std::cout << std::to_integer<uint8_t>(PortC::get()) << std::endl;    

    LcdRS::off();
    std::cout << std::to_integer<uint8_t>(PortC::get()) << std::endl;    
    
    LcdData::set(x);
    std::cout << std::to_integer<uint8_t>(PortC::get()) << std::endl;   

    std::outl<terminal>(std::to_integer<uint8_t>(PortC::get()));
    
    std::cout << "unused: "_pgm << Util::Memory::getUnusedMemory() << std::endl;   
    
    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
