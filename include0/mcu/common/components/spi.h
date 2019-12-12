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
        struct Spi final {
            enum class CtrlA1_t: uint8_t {
                dord = (1 << 6),
                master = (1 << 5),
                clk2x = (1 << 4),
                enable = (1 << 0),
            };
            enum class CtrlA2_t: uint8_t {
                div4 = (0x00 << 1),
                div16 = (0x01 << 1),
                div64 = (0x02 << 1),
                div128 = (0x03 << 1)
            };
            ControlRegister<Spi, Meta::List<CtrlA1_t, CtrlA2_t>> ctrla;

            enum class CtrlB1_t: uint8_t {
                bufen = (1 << 7),
                bufwr = (1 << 6),
                ssd = (1 << 2)
            };
            enum class CtrlB2_t: uint8_t {
                mode0 = 0x00,
                mode1 = 0x01,
                mode2 = 0x02,
                mode3 = 0x03,
            };
            ControlRegister<Spi, Meta::List<CtrlB1_t, CtrlB2_t>> ctrlb;

            enum class IntCtrl_t: uint8_t {
                rxcie = (1 << 7),
                txcie = (1 << 6),
                dreie = (1 << 5),
                ssie  = (1 << 4),
                ie    = (1 << 0),
            };
            ControlRegister<Spi, IntCtrl_t> intctrl;

            enum class IntFlags_t: uint8_t {
                rxcif = (1 << 7),
                  ifl = (1 << 7),
                txcif = (1 << 6),
                wrcol = (1 << 6),
                dreif = (1 << 5),
                ssif  = (1 << 4),
                bufovf= (1 << 0),
            };
            FlagRegister<Spi, IntFlags_t> intflags;
            
            DataRegister<std::byte> data;
            
            template<int N> struct Address;
        };
        static_assert(sizeof(Spi) == 5);
    }
}
