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

#pragma once

#include <stdint.h>
#include "mcu/avr8.h"
#include "mcu/ports.h"

namespace AVR {

template<uint8_t N, typename MCU = DefaultMcuType>
struct AdCompParameter;

template<>
struct AdCompParameter<0, ATMega1284P> {
    AdCompParameter() = delete;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using ain0 = AVR::Pin<PortB, 2>;
    using ain1 = AVR::Pin<PortB, 3>;
};

template<uint8_t N, typename MCU = DefaultMcuType>
class AdComparator final {
    static_assert(N < MCU::AdComparator::count, "wrong adcomparator number"); 
    AdComparator() = delete;
    using parameter = AdCompParameter<N, MCU>;
public:
    static constexpr auto mcuAdComparator = getBaseAddr<typename MCU::AdComparator, N>;
    
    static void init() {
//        mcuAdComparator()->acsr = _BV(ACBG) | _BV(ACI) | _BV(ACIS1) | _BV(ACIS0);
//        mcuAdComparator()->acsr = _BV(ACBG) | _BV(ACIE) | _BV(ACI) | _BV(ACIS1) | _BV(ACIS0);
        mcuAdComparator()->acsr =  _BV(ACIE) | _BV(ACI) | _BV(ACIS1) | _BV(ACIS0);
//        parameter::ain1::template dir<AVR::Input>();
    }
    
private:
};

}
