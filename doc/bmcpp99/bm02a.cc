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

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "external/tle5205.h"
#include "hal/softspimaster.h"
#include "util/disable.h"
#include "simavr/simavrdebugconsole.h"
#include "console.h"

using terminal = SimAVRDebugConsole;

//namespace std {
//    std::basic_ostream<terminal> cout;
//    std::lineTerminator<CRLF> endl;
//}

static constexpr std::hertz pwmFrequency = 1000_Hz;

using tleTimer = AVR::Timer8Bit<0>;

int main() {
    constexpr auto prescaler = AVR::Util::prescalerForAbove<tleTimer>(pwmFrequency);

//    std::cout << prescaler << std::endl;
    
    return prescaler;    
}

