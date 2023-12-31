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
#include <type_traits>
#include "compiler/compiler.h"

namespace AVR {
    struct ATMegaNone;
    struct ATMega8;
    struct ATMega88P;
    struct ATMega168P;
    struct ATMega1284P;
    struct ATMega328P;
    struct ATMega328PB;
    struct ATMega324PB;
    struct ATTiny84;
    struct ATTiny85;
    struct ATTiny25;
}

namespace ARM {
    namespace SAM {
        struct SamC20;
        struct SamC21;
        struct SamD20;
        struct SamD21;
    }
}

#if defined(__AVR_ATmega1284P__)
typedef AVR::ATMega1284P DefaultMcuType;
#elif defined(__AVR_ATmega328P__)
typedef AVR::ATMega328P DefaultMcuType;
#elif defined(__AVR_ATmega88P__)
typedef AVR::ATMega88P DefaultMcuType;
#elif defined(__AVR_ATmega168P__)
typedef AVR::ATMega168P DefaultMcuType;
#elif defined(__AVR_ATmega328PB__)
typedef AVR::ATMega328PB DefaultMcuType;
#elif defined(__AVR_ATmega8__)
typedef AVR::ATMega8 DefaultMcuType;
#elif defined(__AVR_ATmega324PB__)
typedef AVR::ATMega324PB DefaultMcuType;
#elif defined(__AVR_ATtiny85__)
typedef AVR::ATTiny85 DefaultMcuType;
#elif defined(__AVR_ATtiny25__)
typedef AVR::ATTiny25 DefaultMcuType;
#elif defined(__AVR_ATtiny84__)
typedef AVR::ATTiny84 DefaultMcuType;
#elif defined(__SAMD21G18A__)
typedef ARM::SAM::SamD21 DefaultMcuType;
#else
typedef AVR::ATMegaNone DefaultMcuType;
#endif


namespace MCU {
    struct Class8Bit {};
    struct Class16Bit {};
    struct Class32Bit {};
    
    namespace detail {
        template<typename T, typename C, typename Mcu>
        struct is_register_type_base : std::false_type {
            typedef T type;
            typedef C compiler_type;
        };
        template<typename Mcu>
        struct is_register_type_base<uint8_t, Compiler::Gcc, Mcu> : std::true_type{
            typedef uint8_t type;
            typedef Compiler::Gcc compiler_type;
        };
        
        template<typename Mcu>
        struct is_avr {
            inline static constexpr bool value = std::is_same_v<Mcu, AVR::ATMega328P> ||
                                                 std::is_same_v<Mcu, AVR::ATTiny85>;
        };
    }
    
    template<typename T, typename Compiler>
    struct is_register_type : public detail::is_register_type_base<typename std::remove_cv<T>::type, Compiler, DefaultMcuType> {};
    
    template<typename Mcu>
    inline constexpr bool is_avr_v = detail::is_avr<Mcu>::value;
    
    template<bool use = true>
    struct UseInterrupts;
    template<>
    struct UseInterrupts<true> : std::true_type {};
    template<>
    struct UseInterrupts<false> : std::false_type {};

        
}
