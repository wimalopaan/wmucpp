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

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    struct ATMega4809 final {
        template<typename T>
        inline static constexpr bool is_atomic() {return false;}
        
        ATMega4809() = delete;
        
        struct Ram {
            inline static constexpr uintptr_t begin = RAMSTART;  
            inline static constexpr uintptr_t end   = RAMEND;  
        };
        
    };
    template<>
    constexpr bool ATMega4809::is_atomic<uint8_t>() {return true;}
}

namespace std {
//    template<>
//    struct enable_bitmask_operators<AVR::ATMega328PB::Status::Bits> : std::true_type {};
}

namespace AVR {
//    template<>
//    struct ATMega328PB::GPIOR::Address<0> : std::integral_constant<uintptr_t, 0x3e> {};

}
#pragma pack(pop)
