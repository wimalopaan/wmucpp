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

// 8 Mhz, 6k 4ms, 2,7V Brownout
//  sudo avrdude -p attiny84 -P usb -c avrisp2 -U lfuse:w:0xd2:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m

#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "hal/alarmtimer.h"
#include "external/ws2812.h"
#include "mcu/avr/delay.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using ledPin = AVR::Pin<PortB, 4>;
using led = WS2812<2, ledPin>;

int main() 
{
    
    led::init();
    led::off();
    
    while(true) {
        
    }
}
