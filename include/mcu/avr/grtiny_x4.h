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

#include <cstdint>
#include "mcu/avr8.h"

namespace AVR {
    
    template<AVR::ATTiny_X4 MCU>
    struct TimerParameter<0, MCU> {
        using PortA = AVR::Port<typename MCU::PortRegister, AVR::A>;
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        typedef AVR::Pin<PortB, 2> ocAPin;
        typedef AVR::Pin<PortA, 7> ocBPin;
        typedef AVR::Timer8Bit<0, MCU > timer_type;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer8Bit::TCCRA;
        using tb = typename MCU::Timer8Bit::TCCRB;
        static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1;
        static constexpr tb tccrb{0};
        
        static constexpr uint8_t top = 0xff;
    };
    
}
