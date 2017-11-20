/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <type_traits>
#include "compiler/compiler.h"

namespace AVR {
    struct ATMegaNone;
    struct ATMega8;
    struct ATMega88P;
    struct ATMega1284P;
    struct ATMega328P;
    struct ATMega328PB;
    struct ATTiny84;
    struct ATTiny85;
    struct ATTiny25;
}

#if defined(__AVR_ATmega1284P__)
typedef AVR::ATMega1284P DefaultMcuType;
#elif defined(__AVR_ATmega328P__)
typedef AVR::ATMega328P DefaultMcuType;
#elif defined(__AVR_ATmega88P__)
typedef AVR::ATMega88P DefaultMcuType;
#elif defined(__AVR_ATmega328PB__)
typedef AVR::ATMega328PB DefaultMcuType;
#elif defined(__AVR_ATmega8__)
typedef AVR::ATMega8 DefaultMcuType;
#elif defined(__AVR_ATtiny85__)
typedef AVR::ATTiny85 DefaultMcuType;
#elif defined(__AVR_ATtiny25__)
typedef AVR::ATTiny25 DefaultMcuType;
#elif defined(__AVR_ATtiny84__)
typedef AVR::ATTiny84 DefaultMcuType;
#else
typedef AVR::ATMegaNone DefaultMcuType;
#endif


namespace MCU {
    struct Class8Bit {};
    struct Class16Bit {};
    struct Class32Bit {};
    
    template<typename T, typename C, typename Mcu>
    struct is_register_type_base {
        typedef T type;
        typedef C compiler_type;
        inline static constexpr bool value = false;
    };
    template<typename Mcu>
    struct is_register_type_base<uint8_t, Compiler::Gcc, Mcu> {
        typedef uint8_t type;
        typedef Compiler::Gcc compiler_type;
        inline static constexpr bool value = true;
    };
    
    template<typename T, typename Compiler>
    struct is_register_type : public is_register_type_base<typename std::remove_cv<T>::type, Compiler, DefaultMcuType> {};
}