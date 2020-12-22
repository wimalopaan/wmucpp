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
        struct Ccp;

        template<AVR::Concepts::At01DxSeries MCU>
        struct Ccp<MCU> {
            static constexpr auto mcu_cpu = getBaseAddr<typename MCU::Cpu>;
            
            static inline constexpr void unlock(const auto& f) {
                *mcu_cpu()->ccp = 0xd8_B;
                f();
            }
        };
 
//        template<AVR::Concepts::AtTiny1 MCU>
//        struct Ccp<MCU> {
//            static constexpr auto mcu_cpu = getBaseAddr<typename MCU::Cpu>;
            
//            static inline constexpr void unlock(const auto& f) {
//                *mcu_cpu()->ccp = 0xd8_B;
//                f();
//            }
//        };

    }
}
