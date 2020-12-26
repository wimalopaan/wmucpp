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

#pragma once

#include <cstdint>

#include "../common/timer.h"
#include "../common/concepts.h"

namespace AVR {
    template<typename MCU = DefaultMcuType>
    struct Clock;
    
    template<AVR::Concepts::AtDxSeries MCU>
    struct Clock<MCU> {
        using ca_t = typename MCU::Clock::OscHFCtrlA_t;
        
        static inline constexpr auto mcu_clock = getBaseAddr<typename MCU::Clock>;
        static inline constexpr auto prescaler_values = MCU::Clock::prescalerValues;
        
        template<uint8_t P>
        static inline constexpr void prescale() {
            constexpr auto p = AVR::Util::Timer::bitsFrom<P>(prescaler_values);
            mcu_clock()->mclkctrlb.set(p);
        }
        template<uint8_t F>
        static inline constexpr void init() {
            mcu_clock()->oschfctrla.template set<ca_t::f_32mhz>();            
        }
    };
    
    template<AVR::Concepts::At01Series MCU>
    struct Clock<MCU> {
        
        static inline constexpr auto mcu_clock = getBaseAddr<typename MCU::Clock>;
        static inline constexpr auto prescaler_values = MCU::Clock::prescalerValues;
        
        template<uint8_t P>
        static inline constexpr void prescale() {
            constexpr auto p = AVR::Util::Timer::bitsFrom<P>(prescaler_values);
            mcu_clock()->mclkctrlb.set(p);
        }
    };
}
