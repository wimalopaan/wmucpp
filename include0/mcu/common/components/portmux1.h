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
        struct Portmux {
            enum class CtrlA_t : uint8_t {
                evout0_alt1 = (1 << 0)
            };
            ControlRegister<Portmux, CtrlA_t> ctrla;

            enum class CtrlB_t : uint8_t {
                usart0_alt1 = (1 << 0),
                spi0_alt1 = (1 << 2)
            };
            ControlRegister<Portmux, CtrlB_t> ctrlb;

            enum class CtrlC_t : uint8_t {
                tca0_alt1 = (1 << 0),
            };
            ControlRegister<Portmux, CtrlC_t> ctrlc;
            
            static inline constexpr uintptr_t address = 0x0200;
        };
        static_assert(sizeof(Portmux) == 3);
    }
}
