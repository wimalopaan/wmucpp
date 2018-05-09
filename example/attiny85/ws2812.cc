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

#include "config.h"
#include "external/ws2812.h"
#include "mcu/avr/delay.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using led = AVR::Pin<PortB, 3>;

using ws2812_Pin = AVR::Pin<PortB, 4>;
using leds = WS2812<3, ws2812_Pin>;
typedef leds::color_type Color;

int main() 
{
    Set<led>::output();
    led::high();
    
    leds::init();
    leds::off();
    
    uint8_t counter = 0;
    while(true) {
        led::toggle();
        Util::delay(10_ms);
        
        Color color{counter};
        leds::set(color);
        
        ++counter;
    }
}
#ifndef NDEBUG
void assertFunction(bool b, const char* , const char* , unsigned int ) {
   if (!b) {
       while(true) {
       }
   }
}
#endif
