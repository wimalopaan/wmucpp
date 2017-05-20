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

#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"

using namespace AVR;
using PortB = Port<DefaultMcuType::PortRegister, AVR::B>;
using led1 = Pin<PortB, 0>;
using led2 = Pin<PortB, 1>;

using timer1 = AVR::Timer16Bit<1>;

int main() {
    Set<led1>::output();

    timer1::template prescale<1024>();
    
    led1::high();
    while(true) {
        led1::toggle();        
    }    
}
