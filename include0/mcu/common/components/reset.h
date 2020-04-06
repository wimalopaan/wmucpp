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
        struct Reset final {
            enum class Flags_t: uint8_t {
                updi = (0x01 << 5),
                SW   = (0x01 << 4),
                wd   = (0x01 << 3),
                ext  = (0x01 << 2),
                bo   = (0x01 << 1),
                po   = (0x01 << 0),
            };
            FlagRegister<Reset, Flags_t> flags;
            
            enum class Sw_t: uint8_t {
                sw = (0x01 << 0),
            };
            ControlRegister<Reset, Sw_t, WriteOnly> sw;

            static inline constexpr uintptr_t address = 0x0040;
        };
       static_assert(sizeof(Reset) == 2);

    }
}
