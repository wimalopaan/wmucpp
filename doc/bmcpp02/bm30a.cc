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

#define NDEBUG

#include "mcu/ports.h"

using PortB = AVR::Port<AVR::ATMega328P::PortRegister, AVR::B>;
using PortC = AVR::Port<AVR::ATMega328P::PortRegister, AVR::C>;

using Led1 = AVR::Pin<PortB, 1>;
using Led2 = AVR::Pin<PortB, 2>;

struct A{};
using leds = AVR::PinSet<AVR::UsePgmTable, Led1, Led2>;
//using leds = AVR::PinSet<Led1, Led2>;

using din1 = AVR::Pin<PortB, 4>;
using din2 = AVR::Pin<PortB, 5>;

using inputs = AVR::PinSet<din1, din2>;

volatile bitsN_t<2> x;


int main() {
    leds::dir<AVR::Output>();
    leds::allOff();

//    inputs::dir<AVR::Input>();
//    inputs::allPullup();
    
//    leds::on<Led1>();
//    leds::on<Led1, Led2>();
//    leds::off<Led1>();
//    leds::off<Led1, Led2>();
    
//    leds::set<0>();
//    leds::set<1>();
//    leds::set<2>();
//    leds::set<3>();

    leds::set(x);

//    PortC::set(std::byte{1});
    
    
    while(true) {}
}
