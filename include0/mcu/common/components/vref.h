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
#include <std/utility>

namespace AVR {
    namespace Series0 {
        struct Vref{
            enum class CtrlA1_t : uint8_t { 
                adc_V1_1  = (0x01 << 4),
                adc_V2_5  = (0x02 << 4),
                adc_V4_3  = (0x03 << 4),
                adc_V1_5  = (0x04 << 4),
            };
            enum class CtrlA2_t : uint8_t { 
                ac_V0_55 = (0x00 << 0),
                ac_V1_1  = (0x01 << 0),
                ac_V2_5  = (0x02 << 0),
                ac_V4_3  = (0x03 << 0),
                ac_V1_5  = (0x04 << 0),
            };
            ControlRegister<Vref, Meta::List<CtrlA1_t, CtrlA2_t>> ctrla;

            enum class CtrlB_t : uint8_t {
                adc_refen = (1 << 1),
                ac_refen  = (1 << 0),
            };
            ControlRegister<Vref, CtrlB_t> ctrlb;
            
            static inline constexpr uintptr_t address = 0x00a0;
            
        };
    }
}
