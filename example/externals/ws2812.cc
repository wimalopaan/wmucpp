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

#include "config.h"
#include "external/ws2812.h"
#include "mcu/avr/delay.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using ws2812_Pin = AVR::Pin<PortC, 2>;
using leds = WS2812<2, ws2812_Pin>;

typedef leds::color_type Color;

int main() 
{
    leds::init();
    leds::off();
    
    uint8_t counter = 0;
    while(true) {
        Util::delay(10_ms);
        
        Color color{counter};
        leds::set(color);
        
        ++counter;
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
