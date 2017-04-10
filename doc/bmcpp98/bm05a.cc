/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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
#include "mcu/avr/mcutimer.h"
#include "hal/softtimer.h"
#include "simavr/simavrdebugconsole.h"
#include "console.h"

using terminal = SimAVRDebugConsole;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

static constexpr std::hertz f = 100_Hz;

using timer = AVR::Timer8Bit<1>;

int main() {
//    std::cout << "Test"_pgm << std::endl;

    constexpr auto prescaler = AVR::Util::caculateForExactFrequencyAbove<timer>(f);

    std::cout << prescaler.prescaler << std::endl;
    std::cout << prescaler.ocr << std::endl;
    std::cout << prescaler.f << std::endl;
    std::cout << prescaler.isExact << std::endl;

    return prescaler.f.value;
    
//    constexpr auto prescaler2 = AVR::Util::calculate<timer>(f);

//    std::cout << prescaler2.prescaler << std::endl;
//    std::cout << prescaler2.ocr << std::endl;
//    std::cout << prescaler2.f << std::endl;
//    std::cout << prescaler2.isExact << std::endl;
    
}

