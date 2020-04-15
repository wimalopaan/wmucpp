/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <std/array>

namespace AVR {
    namespace detail {
        namespace Uninit {
            static inline volatile std::byte g_value __attribute__ ((section (".noinit")));
            static inline volatile uint16_t g_counter __attribute__ ((section (".noinit")));
        }
    }
    
    template<typename ValueType = std::byte, typename MCU = DefaultMcuType>
    struct Uninitialized {
        static inline void reset() {
            value = 0x00_B;
            counter = 0;
        }
        static inline auto& value   = detail::Uninit::g_value;
        static inline auto& counter = detail::Uninit::g_counter;
        
        // geht nicht
        
//        static inline volatile ValueType value __attribute__ ((section (".noinit")));
    }; 
}
