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
#include <cstddef>
#include <array>

namespace Hal {
    template<typename FlagRegister, uint8_t StartBit, uint8_t LastBit = StartBit>
    requires requires() {
        FlagRegister::get();
    }
    struct Flag final {
        inline static constexpr uint8_t startBit = StartBit;
        inline static constexpr uint8_t lastBit = LastBit;
        Flag() = delete;
        static_assert(StartBit <= LastBit, "wrong ordering");
        static_assert(StartBit < 8, "wrong start bit in flag register"); // todo: FlagRegister::size
        static_assert(LastBit < 8, "wrong end bit in flag register");
        
        template<uint8_t N = 0>                                  
        static void set() {
            static_assert(N <= (LastBit - StartBit), "wrong bit in flag");
            FlagRegister::get() |= std::byte(1 << (StartBit + N));
        }  
        template<uint8_t N = 0>                                  
        static void reset() {
            static_assert(N <= (LastBit - StartBit), "wrong bit in flag");
            FlagRegister::get() &= ~std::byte(1 << (StartBit + N));
        }  
        template<uint8_t N = 0>                                  
        static bool isSet() {
            static_assert(N <= (LastBit - StartBit), "wrong bit in flag");
            return std::any(FlagRegister::get() & std::byte(1 << (StartBit + N)));                  
        }
    };
}
