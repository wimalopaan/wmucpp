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

#include "../common/concepts.h"

namespace AVR {
    template<uint8_t N, typename MCU = DefaultMcuType>
    struct DAC;
    
    template<uint8_t N, AVR::Concepts::AtTiny1 MCU>
    struct DAC<N, MCU> {
        using ca_t = typename MCU::Dac::CtrlA_t;
        using va_t = typename MCU::Vref::CtrlA2_t;
        using vb_t = typename MCU::Vref::CtrlB_t;
        
        static inline constexpr auto mcu_dac = getBaseAddr<typename MCU::Dac, N>;
        static inline constexpr auto mcu_vref = AVR::getBaseAddr<typename MCU::Vref>;
        
        static inline void init() {
            mcu_vref()->ctrla.template setPartial(va_t::dac0_V2_5);           
            mcu_vref()->ctrlb.template add<vb_t::dac0_refen>();           
            mcu_dac()->ctrla.template set<ca_t::outen | ca_t::enable>();
        }
        
        static inline void put(const uint8_t v) {
            *mcu_dac()->data = std::byte{v};
        }
    };
}
