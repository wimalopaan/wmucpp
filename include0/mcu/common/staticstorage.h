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
    
    template<etl::Concepts::NamedConstant N, typename ValueType = std::byte>
    struct StaticByte {
        using value_type = ValueType;
        inline static value_type& raw() {
            return value;
        }
    private:
        static inline value_type value;
    };
    
    template<etl::Concepts::NamedConstant N, etl::Concepts::NamedConstant NumberOfBits>
    struct StaticBoolArray {
        inline static constexpr uint8_t size() {
            return NumberOfBits::value;
        }
        template<uint8_t M>
        inline static constexpr bool& bit() {
            static_assert(M < NumberOfBits::value);
            return mBits[M];
        }
    private:    
        static inline std::array<bool, NumberOfBits::value> mBits;
    };
    
}
