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
        using cb_t = typename MCU::Clock::MClkCtrlB_t;
        
        static inline constexpr auto mcu_clock = getBaseAddr<typename MCU::Clock>;
        static inline constexpr auto prescaler_values = MCU::Clock::prescalerValues;
        
        template<uint8_t P>
        static inline constexpr void prescale() {
            constexpr auto p = AVR::Util::Timer::bitsFrom<P>(prescaler_values);
            mcu_clock()->mclkctrlb.set(p);
        }
        template<External::Units::megahertz F>
        static inline constexpr void init() {
            if constexpr(F.value == 32) {
                mcu_clock()->oschfctrla.template set<ca_t::f_32mhz>();            
            }
            else if constexpr(F.value == 28) {
                mcu_clock()->oschfctrla.template set<ca_t::f_28mhz>();            
            }
            else if constexpr(F.value == 24) {
                mcu_clock()->oschfctrla.template set<ca_t::f_24mhz>();            
            }
            else {
                static_assert(std::false_v<MCU>, "wrong frequency");
            }
            mcu_clock()->mclkctrlb.set(cb_t{0});
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
