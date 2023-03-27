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
        struct Cpu final {
            volatile uint8_t padding[4];
            DataRegister<Cpu, ReadWrite, std::byte> ccp;
            
            volatile std::byte reserved[0x0c - 0x05 + 1];
            
            volatile uint16_t sp;
            
            enum class SReg_t: uint8_t {
                globalIntEnable = (1 << 7), 
                bitCopy = (1 << 6) 
            };
            ControlRegister<Cpu, SReg_t> sreg;
            
            static inline constexpr uintptr_t address = 0x0030;
        };
       static_assert(sizeof(Cpu) == 16);
    }
}
