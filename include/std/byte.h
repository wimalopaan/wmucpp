/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include <std/concepts.h>

namespace std {

enum class byte : unsigned char {};

template<std::Integral T>
constexpr T to_integer(byte b) noexcept {
    return T((uint8_t)b);
}

constexpr bool any(std::byte b) {
    return b != std::byte{0};
}
constexpr bool none(std::byte b) {
    return b == std::byte{0};
}

template<typename IType>
constexpr byte& operator<<=(byte& b, IType shift) noexcept {
    return b = byte(static_cast<uint8_t>(b) << shift);
}
template<typename IType>
constexpr byte& operator>>=(byte& b, IType shift) noexcept {
    return b = byte(static_cast<uint8_t>(b) >> shift);
}

template<typename IType>
constexpr byte operator>>(byte b, IType shift) noexcept {
    return byte(static_cast<uint8_t>(b) >> shift);
}
template<typename IType>
constexpr byte operator<<(byte b, IType shift) noexcept {
    return byte(static_cast<uint8_t>(b) << shift);
}

constexpr byte operator&(byte a, byte b) {
    return byte{static_cast<uint8_t>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b))};
}
constexpr byte operator|(byte a, byte b) {
    return byte{static_cast<uint8_t>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b))};
}
constexpr byte operator^(byte a, byte b) {
    return byte{static_cast<uint8_t>(static_cast<uint8_t>(a) ^ static_cast<uint8_t>(b))};
}
void operator&=(volatile byte& a, byte b) {
    a = byte{static_cast<uint8_t>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b))};
}
constexpr byte& operator|=(byte& a, byte b) {
    return a = byte{static_cast<uint8_t>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b))};
}
void operator|=(volatile byte& a, byte b) {
    a = byte{static_cast<uint8_t>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b))};
}
void operator^=(volatile byte& a, byte b) {
    a = byte{static_cast<uint8_t>(static_cast<uint8_t>(a) ^ static_cast<uint8_t>(b))};
}
constexpr byte operator~(byte b) {
    return b = byte(~static_cast<uint8_t>(b));
}

} // std

constexpr std::byte operator"" _B(unsigned long long v) {
    return std::byte{static_cast<uint8_t>(v)};
}
