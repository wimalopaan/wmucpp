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
    namespace Cpu {

        template<typename MCU = DefaultMcuType>
        struct SysCfg;

        template<AVR::Concepts::AtDxSeries MCU>
        struct SysCfg<MCU> {
            static constexpr auto mcu_cfg = getBaseAddr<typename MCU::SysCfg>;
            
            static inline uint8_t major() {
                return (uint8_t)((*mcu_cfg()->devId >> 4) & 0x0f_B);
            }
            static inline uint8_t minor() {
                return (uint8_t)(*mcu_cfg()->devId & 0x0f_B);
            }
            static inline std::byte id () {
                return std::byte((('a' + major()) << 4) + minor());
            }
        };
 
    }
}
