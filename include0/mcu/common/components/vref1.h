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
    namespace Series1 {
        struct Vref{
            enum class CtrlA1_t : uint8_t { 
                adc0_V0_55  = (0x00 << 4),
                adc0_V1_1  = (0x01 << 4),
                adc0_V2_5  = (0x02 << 4),
                adc0_V4_3  = (0x03 << 4),
                adc0_V1_5  = (0x04 << 4),
            };
            enum class CtrlA2_t : uint8_t { 
                dac0_V0_55 = (0x00 << 0),
                dac0_V1_1  = (0x01 << 0),
                dac0_V2_5  = (0x02 << 0),
                dac0_V4_3  = (0x03 << 0),
                dac0_V1_5  = (0x04 << 0),
            };
            
            ControlRegister<Vref, Meta::List<CtrlA1_t, CtrlA2_t>> ctrla;

            enum class CtrlB_t : uint8_t {
                dac2_refen = (1 << 5),
                adc1_refen = (1 << 4),
                dac1_refen = (1 << 3),
                adc0_refen = (1 << 1),
                dac0_refen  = (1 << 0),
            };
            ControlRegister<Vref, CtrlB_t> ctrlb;

            enum class CtrlC1_t : uint8_t { 
                adc1_V0_55  = (0x00 << 4),
                adc1_V1_1  = (0x01 << 4),
                adc1_V2_5  = (0x02 << 4),
                adc1_V4_3  = (0x03 << 4),
                adc1_V1_5  = (0x04 << 4),
            };
            enum class CtrlC2_t : uint8_t { 
                dac1_V0_55 = (0x00 << 0),
                dac1_V1_1  = (0x01 << 0),
                dac1_V2_5  = (0x02 << 0),
                dac1_V4_3  = (0x03 << 0),
                dac1_V1_5  = (0x04 << 0),
            };
            
            ControlRegister<Vref, Meta::List<CtrlC1_t, CtrlC2_t>> ctrlc;

            enum class CtrlD_t : uint8_t { 
                dac2_V0_55 = (0x00 << 0),
                dac2_V1_1  = (0x01 << 0),
                dac2_V2_5  = (0x02 << 0),
                dac2_V4_3  = (0x03 << 0),
                dac2_V1_5  = (0x04 << 0),
            };
            
            ControlRegister<Vref, Meta::List<CtrlD_t>> ctrld;
            
            static inline constexpr uintptr_t address = 0x00a0;
            
        };
    }
    namespace detail {
        template<> struct register_bit_position<Series1::Vref::CtrlA1_t> : std::integral_constant<uint8_t, 4> {};
        template<> struct register_bit_position<Series1::Vref::CtrlA2_t> : std::integral_constant<uint8_t, 0> {};
        template<> struct register_bit_mask<Series1::Vref::CtrlA1_t> : std::integral_constant<std::byte, 0xf0_B> {};
        template<> struct register_bit_mask<Series1::Vref::CtrlA2_t> : std::integral_constant<std::byte, 0x0f_B> {};
    }
}
