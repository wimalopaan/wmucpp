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

#include "blink.h"

#include "mcu/ports.h"
#include "mcu/avr/delay.h"

using namespace AVR;
using PortB = Port<DefaultMcuType::PortRegister, AVR::B>;
using led = Pin<PortB, 0>;

// externer takt
// sudo avrdude -p atmega8 -P usb -c avrisp2 -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m

int main()
{
    Set<led>::output();
    led::dir<Output>();        

    led::high();
    while(true) {
        Util::delay(500_us);
        led::toggle();        
    }    
}
