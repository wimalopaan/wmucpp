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

#include "vref.h"

namespace AVR {
    namespace Series0 {
        struct AdComparator {
            enum class CtrlA1_t : uint8_t {
                standby = (1 << 7),
                outen   = (1 << 6),
                low_power   = (1 << 3),
                enable = 1 << 0
            };
            enum class CtrlA2_t : uint8_t {
                bothedge = (0x00 << 4),
                negative_edge = (0x02 << 4),
                positive_edge = (0x03 << 4),
            };
            enum class CtrlA3_t : uint8_t {
                hyst_none = (0x00 << 1),
                hyst_small = (0x01 << 1),
                hyst_medium = (0x02 << 1),
                hyst_large = (0x03 << 1),
            };
            ControlRegister<AdComparator, Meta::List<CtrlA1_t, CtrlA2_t, CtrlA3_t>> ctrla;

            volatile std::byte reserved1;

            enum class MuxCtrl1_t : uint8_t {
                invert_output = (1 << 7),
            };
            enum class MuxCtrl2_t : uint8_t {
                ainp0 = (0x00 << 3),
                ainp1 = (0x01 << 3),
                ainp2 = (0x02 << 3),
                ainp3 = (0x03 << 3),
            };
            enum class MuxCtrl3_t : uint8_t {
                ainn0 = (0x00 << 0),
                ainn1 = (0x01 << 0),
                ainn2 = (0x02 << 0),
                dacref = (0x03 << 0),
            };
            ControlRegister<AdComparator, Meta::List<MuxCtrl1_t, MuxCtrl2_t, MuxCtrl3_t>> muxctrl;
            
            volatile std::byte reserved2;
            
            DataRegister<AdComparator, ReadWrite, uint8_t> dacref;
            
            volatile std::byte reserved3;

            enum class IntCtrl_t : uint8_t {
                cmp = (1 << 0)
            };
            ControlRegister<AdComparator, IntCtrl_t> intctrl;
            
            enum class Status_t : uint8_t {
                state = (1 << 4),
                cmp = (1 << 0)
            };
            FlagRegister<AdComparator, Status_t> status;

            template<int N> struct Address;
        };
        static_assert(sizeof(AdComparator) == 0x08);
    }
    namespace detail {
        template<> struct register_bit_position<Series0::AdComparator::CtrlA1_t> : std::integral_constant<uint8_t, 0> {};
        template<> struct register_bit_position<Series0::AdComparator::CtrlA2_t> : std::integral_constant<uint8_t, 4> {};
        template<> struct register_bit_position<Series0::AdComparator::CtrlA3_t> : std::integral_constant<uint8_t, 1> {};

        template<> struct register_bit_mask<Series0::AdComparator::CtrlA1_t> : std::integral_constant<std::byte, (0b11001001_B) > {};
        template<> struct register_bit_mask<Series0::AdComparator::CtrlA2_t> : std::integral_constant<std::byte, (0x03_B << 4) > {};
        template<> struct register_bit_mask<Series0::AdComparator::CtrlA3_t> : std::integral_constant<std::byte, (0x03_B << 1) > {};

        template<> struct register_bit_position<Series0::AdComparator::MuxCtrl1_t> : std::integral_constant<uint8_t, 7> {};
        template<> struct register_bit_position<Series0::AdComparator::MuxCtrl2_t> : std::integral_constant<uint8_t, 3> {};
        template<> struct register_bit_position<Series0::AdComparator::MuxCtrl3_t> : std::integral_constant<uint8_t, 0> {};

        template<> struct register_bit_mask<Series0::AdComparator::MuxCtrl1_t> : std::integral_constant<std::byte, (0x01_B << 7) > {};
        template<> struct register_bit_mask<Series0::AdComparator::MuxCtrl2_t> : std::integral_constant<std::byte, (0x03_B << 3) > {};
        template<> struct register_bit_mask<Series0::AdComparator::MuxCtrl3_t> : std::integral_constant<std::byte, (0x03_B << 0) > {};
    }
}
