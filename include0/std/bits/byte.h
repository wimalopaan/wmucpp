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

#include "bits/bitmask.h"

namespace std {
    
    enum class byte : uint8_t {};
    
    template<>
    struct enable_bitmask_operators<byte> : true_type {};
    
    template<typename T = uint8_t>
    constexpr T to_integer(byte b) noexcept {
        return T((uint8_t)b);
    }
    
    constexpr bool any(std::byte b) noexcept {
        return b != std::byte{0};
    }
    constexpr bool none(std::byte b) noexcept {
        return b == std::byte{0};
    }

    
    template<typename E>
    constexpr bool isset(E flags) {
        typedef typename std::underlying_type<E>::type underlying;
        return static_cast<underlying>(flags) != 0;
    }
    
} // std

constexpr std::byte operator"" _B(unsigned long long v) {
    return std::byte{static_cast<uint8_t>(v)};
}
constexpr std::byte operator"" _B(char v) {
    return std::byte{v};
}
