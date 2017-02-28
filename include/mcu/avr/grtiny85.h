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

namespace AVR {

template<uint8_t TimerN, typename MCU = DefaultMcuType>
struct TimerParameter;

template<>
struct TimerParameter<0, ATTiny85> {
    using PortD = AVR::Port<ATTiny85::PortRegister, AVR::D>;
    typedef AVR::Pin<PortD, 6> ocAPin;
    typedef AVR::Pin<PortD, 5> ocBPin;
    typedef AVR::Timer8Bit<0, ATTiny85> timer_type;
    typedef timer_type::value_type value_type;
    
    using ta = AVR::ATTiny85::Timer8Bit::TCCRA;
    using tb = AVR::ATTiny85::Timer8Bit::TCCRB;
    
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1;
    static constexpr tb tccrb{0};
    
    
    static constexpr uint8_t top = 0xff;
};

}