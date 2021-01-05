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

    template<AVR::Concepts::AtDxSeries MCU>
    struct SigRow<MCU> {
        static inline constexpr auto mcu_sigrow = getBaseAddr<typename MCU::SigRow>;
        
        static inline auto id() {
            std::array<std::byte, 3> data;
            data[2] = *mcu_sigrow()->deviceId2;
            data[1] = *mcu_sigrow()->deviceId1;
            data[0] = *mcu_sigrow()->deviceId0;
            return data;
        }
        
        static inline uint16_t toffset() {
            return *mcu_sigrow()->tempSense1;
        }
        static inline uint16_t tgain() {
            return *mcu_sigrow()->tempSense0;
        }
        
        static inline constexpr External::Units::celsius<uint16_t, std::ratio<1,1>> adcValueToTemperature(const etl::uint_ranged<uint16_t, 0, 4095>& v) {
            const uint16_t offset = toffset();
            const uint16_t gain   = tgain();
            
            uint32_t t = offset - v.toInt() * 2;
            t *= gain;
            t += 0x0800;
            t >>= 12;
            t -= 273;
            return {uint16_t(t)};
        }
        template<typename R, uint8_t Offset, typename VRef = AVR::Vref::V2_048>
        static inline constexpr External::Units::celsius<uint16_t, R> adcValueToTemperature(const etl::uint_ranged<uint16_t, 0, 4095>& v) {
            constexpr uint16_t divider = 4096 / R::denom;
//            std::integral_constant<uint16_t, divider>::_;
            constexpr uint16_t offset1  = (273 - Offset) * R::denom;
            
            const uint16_t offset = toffset();
            const uint16_t gain   = tgain();
            
            if constexpr(std::is_same_v<VRef, AVR::Vref::V2_048>) {
                uint32_t t = offset - v.toInt();
                t *= gain;
                t += 0x0800;
                t /= divider;
                t -= offset1;
                return {uint16_t(t)};
            }
            else if constexpr(std::is_same_v<VRef, AVR::Vref::V4_096>) {
                uint32_t t = offset - v.toInt() * 2;
                t *= gain;
                t += 0x0800;
                t /= divider;
                t -= offset1;
                return {uint16_t(t)};
            }
            else {
                static_assert(std::false_v<VRef>);
            }
        }
    };

    
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
