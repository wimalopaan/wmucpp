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
#include <etl/types.h>

#include "../external/units/physical.h"
#include "../common/concepts.h"

namespace AVR {
    template<typename MCU = DefaultMcuType>
    struct SigRow;
    
    template<AVR::Concepts::At01Series MCU>
    struct SigRow<MCU> {
        static inline constexpr auto mcu_sigrow = getBaseAddr<typename MCU::SigRow>;
        
        static inline constexpr External::Units::celsius<uint16_t, std::ratio<1,1>> adcValueToTemperature(const etl::uint_ranged<uint16_t, 0, 1023>& v) {
            int8_t offset = (int8_t)*mcu_sigrow()->tempSense1;
            uint8_t gain = (uint8_t)*mcu_sigrow()->tempSense0;
            
            uint32_t t = v.toInt() - offset;
            t *= gain;
            t += 0x80;
            t >>= 8;
            t -= 273;
            return {uint16_t(t)};
        }
        template<typename R, uint8_t Offset>
        static inline constexpr External::Units::celsius<uint16_t, R> adcValueToTemperature(const etl::uint_ranged<uint16_t, 0, 1023>& v) {
            constexpr uint16_t divider = 256 / R::denom;
//            std::integral_constant<uint16_t, divider>::_;
            constexpr uint16_t offset1  = (273 - Offset) * R::denom;
            int8_t offset = (int8_t)*mcu_sigrow()->tempSense1;
            uint8_t gain = (uint8_t)*mcu_sigrow()->tempSense0;
            
            uint32_t t = v.toInt() - offset;
            t *= gain;
            t += 0x80;
            t /= divider;
            t -= offset1;
            return {uint16_t(t)};
        }
    };
}
