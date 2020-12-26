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
    namespace SeriesDa {
        struct Vref {
            enum class VRef_t : uint8_t { 
                V1024  = 0x00,
                V2048  = 0x01,
                V4096  = 0x02,
                V2500  = 0x03,
                Vdd    = 0x05,
                Vext   = 0x06,
                on     = 0x80
            };
            
            ControlRegister<Vref, VRef_t> adc0ref;

            volatile const std::byte reserved1;
            
            ControlRegister<Vref, VRef_t> dac0ref;

            volatile const std::byte reserved2;
            
            ControlRegister<Vref, VRef_t> acref;

            static inline constexpr uintptr_t address = 0x00a0;
            
        };
    }
}
