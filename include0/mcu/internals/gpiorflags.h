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

#include <cstdint>
#include <cstddef>

#include <etl/concepts.h>

namespace AVR {
    template<typename StaticByteStorage, etl::Concepts::NamedConstant StartBit, etl::Concepts::NamedConstant = StartBit>
    struct GPIORFlags;
    
    template<typename StaticFlagStorage, etl::Concepts::NamedConstant StartBit, etl::Concepts::NamedConstant LastBit>
    requires requires() {
        StaticFlagStorage::raw();
    }
    struct GPIORFlags<StaticFlagStorage, StartBit, LastBit> final {
        GPIORFlags() = delete;
        using storage = StaticFlagStorage;
        inline static constexpr uint8_t startBit = StartBit::value;
        inline static constexpr uint8_t lastBit = LastBit::value;
        static_assert(startBit <= lastBit, "wrong ordering");
        static_assert(lastBit < 8, "wrong end bit in flag register");
        
        template<uint8_t N = 0>                                  
        static void set() {
            static_assert(N <= (lastBit - startBit), "wrong bit in flag");
            storage::raw() |= std::byte(1 << (startBit + N));
        }  
        template<uint8_t N = 0>                                  
        static void reset() {
            static_assert(N <= (lastBit - startBit), "wrong bit in flag");
            storage::raw() &= ~std::byte(1 << (startBit + N));
        }  
        template<uint8_t N = 0>                                  
        static bool isSet() {
            static_assert(N <= (lastBit - startBit), "wrong bit in flag");
            return std::any(storage::raw() & std::byte(1 << (startBit + N)));                  
        }
    };

    template<typename StaticFlagStorage, etl::Concepts::NamedConstant StartBit, etl::Concepts::NamedConstant LastBit>
    requires requires() {
        StaticFlagStorage::size();
        StaticFlagStorage::template bit<0>();
    }
    struct GPIORFlags<StaticFlagStorage, StartBit, LastBit> final {
        GPIORFlags() = delete;
        using storage = StaticFlagStorage;
        inline static constexpr uint8_t startBit = StartBit::value;
        inline static constexpr uint8_t lastBit = LastBit::value;
        static_assert(startBit <= lastBit, "wrong ordering");
        static_assert(lastBit < storage::size(), "wrong end bit in flag register");
        
        template<uint8_t N = 0>                                  
        static void set() {
            static_assert(N <= (lastBit - startBit), "wrong bit in flag");
            storage::template bit<startBit + N>() = true;
        }  
        template<uint8_t N = 0>                                  
        static void reset() {
            static_assert(N <= (lastBit - startBit), "wrong bit in flag");
            storage::template bit<startBit + N>() = false;
        }  
        template<uint8_t N = 0>                                  
        static bool isSet() {
            static_assert(N <= (lastBit - startBit), "wrong bit in flag");
            return storage::template bit<startBit + N>();
        }
    };
}
