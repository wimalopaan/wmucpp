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

#include "mcu/avr8.h"
#include "mcu/avr/util.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "util/bits.h"
#include "std/algorithm.h"

int main() {
    using tt = DefaultMcuType::Timer8Bit;
    using tfa = tt::TCCRA;
    using tfb = tt::TCCRB;
    
    auto t0 = AVR::getBaseAddr<tt, 0>();
    t0->tccrb.set(tfb::cs0 | tfb::cs1);
            
    constexpr auto tsd = AVR::Util::calculate<AVR::Timer8Bit<0>>(1000_Hz);
    
    AVR::Timer8Bit<0>::prescale<tsd.prescaler>();    
    
    while(true) {}
}
