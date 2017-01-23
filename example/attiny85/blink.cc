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

// 16 Mhz PLL
// sudo avrdude -p attiny85 -P usb -c avrisp2 -U -U lfuse:w:0xf1:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

#include "mcu/ports.h"

using namespace AVR;
using PortB = Port<DefaultMcuType::PortRegister, AVR::B>;
using led = Pin<PortB, 3>;

int main()
{
    Set<led>::output();

    led::high();
    while(true) {
        led::toggle();        
    }    
}
