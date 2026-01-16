/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

namespace Mcu {
    template<bool F>
    using UseInterrupts = std::integral_constant<bool, F>;
    
    struct Output;
    struct Input;
    
    struct High;
    
    namespace Stm {
        template<typename MCU> struct isG4xx : std::false_type {};
        template<typename MCU> struct isG0xx : std::false_type {};
        template<> struct isG4xx<Stm32G431> : std::true_type {};
        template<> struct isG4xx<Stm32G473> : std::true_type {};

        template<> struct isG0xx<Stm32G030> : std::true_type {};
        template<> struct isG0xx<Stm32G031> : std::true_type {};
        template<> struct isG0xx<Stm32G0B1> : std::true_type {};
        template<> struct isG0xx<Stm32G051> : std::true_type {};

        template<typename MCU> concept G4xx = isG4xx<MCU>::value;
        template<typename MCU> concept G0xx = isG0xx<MCU>::value;

        struct A {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 0;
            static inline constexpr uint8_t iopBit = 0x01 << 0;
        };    
        struct B {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 1;
            static inline constexpr uint8_t iopBit = 0x01 << 1;
        };
        struct C {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 2;
            static inline constexpr uint8_t iopBit = 0x01 << 2;
        };
        struct D {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 3;
            static inline constexpr uint8_t iopBit = 0x01 << 3;
        };
        struct E {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 4;
            static inline constexpr uint8_t iopBit = 0x01 << 4;
        };
        struct F {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 5;
            static inline constexpr uint8_t iopBit = 0x01 << 5;
        };
        
        template<typename C> struct Address;
    }
}
